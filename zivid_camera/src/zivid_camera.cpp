#include "zivid_camera.hpp"
#include "CaptureGeneralConfigUtils.h"
#include "CaptureFrameConfigUtils.h"

#include "zivid_camera/CameraInfoModelName.h"
#include "zivid_camera/CameraInfoSerialNumber.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/config_tools.h>

#include <Zivid/HDR.h>
#include <Zivid/Firmware.h>

#include <boost/algorithm/string.hpp>

#include <sstream>

namespace
{
sensor_msgs::PointField createPointField(std::string name, uint32_t offset, uint8_t datatype, uint32_t count)
{
  sensor_msgs::PointField point_field;
  point_field.name = name;
  point_field.offset = offset;
  point_field.datatype = datatype;
  point_field.count = count;
  return point_field;
}

bool big_endian()
{
  union
  {
    uint32_t i;
    char c[4];
  } u = { 0x01020304 };
  return u.c[0] == 0x01;
}

template <class T>
void fillCommonMsgFields(T& msg, const std_msgs::Header& header, const Zivid::PointCloud& pc)
{
  msg.header = header;
  msg.height = pc.height();
  msg.width = pc.width();
  msg.is_bigendian = big_endian();
}

template <class T>
ros::ServiceServer advertiseService(ros::NodeHandle& nh, const std::string& name,
                                    boost::function<bool(typename T::Request&, typename T::Response&)> cb)
{
  ROS_INFO("Advertisting service '%s'", name.c_str());
  return nh.advertiseService(name, cb);
}

}  // namespace

zivid_camera::ZividCamera::ZividCamera(ros::NodeHandle& nh)
  : nh_(nh)
  , priv_("~")
  , currentCaptureGeneralConfig_(decltype(currentCaptureGeneralConfig_)::__getDefault__())
  , image_transport_(nh_)
  , header_seq_(0)
{
  ROS_INFO("Zivid ROS driver version %s", ZIVID_ROS_DRIVER_VERSION);

  ROS_INFO("Node's namespace is '%s'", nh_.getNamespace().c_str());
  if (ros::this_node::getNamespace() == "/")
  {
    // Require the user to specify the namespace that this node will run in.
    // See REP-135 http://www.ros.org/reps/rep-0135.html
    throw std::runtime_error("Zivid driver started in the global namespace ('/')! This is unsupported. "
                             "Please specify namespace, fex. using the ROS_NAMESPACE environment variable.");
  }

  ROS_INFO("Built towards Zivid API version %s", ZIVID_VERSION);
  ROS_INFO("Running with Zivid API version %s", Zivid::Version::libraryVersion().c_str());
  if (Zivid::Version::libraryVersion() != ZIVID_VERSION)
  {
    throw std::runtime_error("Zivid library mismatch! The running Zivid version does not match the "
                             "version this library was built towards. Hint: Try to clean and re-build your project "
                             "from scratch.");
  }

  std::string serial_number;
  priv_.param<decltype(serial_number)>("serial_number", serial_number, "");

  int num_capture_frames;
  priv_.param<decltype(num_capture_frames)>("num_capture_frames", num_capture_frames, 10);

  priv_.param<decltype(frame_id_)>("frame_id", frame_id_, "zivid_optical_frame");

  std::string file_camera_path;
  priv_.param<decltype(file_camera_path)>("file_camera_path", file_camera_path, "");
  const bool file_camera_mode = !file_camera_path.empty();

  if (file_camera_mode)
  {
    ROS_INFO("Creating file camera from file '%s'", file_camera_path.c_str());
    camera_ = zivid_.createFileCamera(file_camera_path);
  }
  else
  {
    auto cameras = zivid_.cameras();
    ROS_INFO("%zu cameras found", cameras.size());
    if (cameras.empty())
    {
      throw std::runtime_error("No cameras found. Ensure that the camera is connected to the USB3 port on your PC.");
    }
    else if (serial_number.empty())
    {
      camera_ = [&]() {
        ROS_INFO("Selecting first available camera");
        for (auto& c : cameras)
        {
          if (c.state().isAvailable())
            return c;
        }
        throw std::runtime_error("No available cameras found. Is the camera in use by another process?");
      }();
    }
    else
    {
      if (serial_number.find(":") == 0)
      {
        serial_number = serial_number.substr(1);
      }
      camera_ = [&]() {
        ROS_INFO("Searching for camera with serial number '%s' ...", serial_number.c_str());
        for (auto& c : cameras)
        {
          if (c.serialNumber() == Zivid::SerialNumber(serial_number))
            return c;
        }
        throw std::runtime_error("No camera found with serial number '" + serial_number + "'");
      }();
    }

    if (!Zivid::Firmware::isUpToDate(camera_))
    {
      ROS_INFO("The camera firmware is not up-to-date, starting update");
      Zivid::Firmware::update(camera_, [](double progress, const std::string& state) {
        ROS_INFO("  [%.0f%%] %s", progress, state.c_str());
      });
      ROS_INFO("Firmware update completed");
    }
  }

  ROS_INFO("%s", camera_.toString().c_str());
  if (!file_camera_mode)
  {
    ROS_INFO("Connecting to camera ...");
    camera_.connect();
  }
  ROS_INFO("Connected to camera");

  const auto cameraSettings = camera_.settings();
  setupCaptureGeneralConfigNode(cameraSettings);

  ROS_INFO("Setting up %d capture_frame dynamic_reconfigure nodes", num_capture_frames);
  for (int i = 0; i < num_capture_frames; i++)
  {
    setupCaptureFrameConfigNode(i, cameraSettings);
  }

  ROS_INFO("Advertising topics");
  point_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("depth/points", 1);
  rgb_image_publisher_ = image_transport_.advertise("rgb/image_color", 1);
  depth_image_publisher_ = image_transport_.advertise("depth/image", 1);

  ROS_INFO("Advertising services");

  capture_service_ = advertiseService<zivid_camera::Capture>(
      nh_, "capture", [this](auto& req, auto& res) { return captureServiceHandler(req, res); });

  camera_info_model_name_service_ =
      advertiseService<zivid_camera::CameraInfoModelName>(nh_, "camera_info/model_name", [this](auto&, auto& res) {
        res.model_name = camera_.modelName();
        return true;
      });

  camera_info_serial_number_service_ = advertiseService<zivid_camera::CameraInfoSerialNumber>(
      nh_, "camera_info/serial_number", [this](auto&, auto& res) {
        res.serial_number = camera_.serialNumber().toString();
        return true;
      });

  ROS_INFO("Zivid camera node is now ready!");
}

zivid_camera::ZividCamera::~ZividCamera() = default;

void zivid_camera::ZividCamera::setupCaptureGeneralConfigNode(const Zivid::Settings& defaultSettings)
{
  capture_general_dr_server_ = std::make_unique<dynamic_reconfigure::Server<zivid_camera::CaptureGeneralConfig>>(
      capture_general_dr_server_mutex_, ros::NodeHandle(nh_, "capture_general"));

  // Setup min, max, default and current config
  const auto minConfig = getCaptureGeneralConfigMinFromZividSettings(defaultSettings);
  capture_general_dr_server_->setConfigMin(minConfig);

  const auto maxConfig = getCaptureGeneralConfigMaxFromZividSettings(defaultSettings);
  capture_general_dr_server_->setConfigMax(maxConfig);

  const auto defaultConfig = getCaptureGeneralConfigDefaultFromZividSettings(defaultSettings);
  capture_general_dr_server_->setConfigDefault(defaultConfig);
  capture_general_dr_server_->updateConfig(defaultConfig);

  // Setup the cb, this will invoke the cb, ensuring that the locally cached
  // config is updated.
  capture_general_dr_server_->setCallback(
      boost::bind(&zivid_camera::ZividCamera::onCaptureGeneralConfigChanged, this, _1, _2));
}

void zivid_camera::ZividCamera::setupCaptureFrameConfigNode(int nodeIdx, const Zivid::Settings& defaultSettings)
{
  auto frame_config = std::make_unique<DRFrameConfig>("capture_frame/frame_" + std::to_string(nodeIdx), nh_);

  // Setup min, max, default and current config
  const auto minConfig = getCaptureFrameConfigMinFromZividSettings(defaultSettings);
  frame_config->dr_server.setConfigMin(minConfig);

  const auto maxConfig = getCaptureFrameConfigMaxFromZividSettings(defaultSettings);
  frame_config->dr_server.setConfigMax(maxConfig);

  const auto defaultConfig = getCaptureFrameConfigDefaultFromZividSettings(defaultSettings);
  frame_config->dr_server.setConfigDefault(defaultConfig);
  frame_config->dr_server.updateConfig(defaultConfig);

  // Setup the cb, this will invoke the cb, ensuring that frame_config.config is updated.
  frame_config->dr_server.setCallback(boost::bind(&zivid_camera::ZividCamera::onCaptureFrameConfigChanged, this, _1, _2,
                                                  std::ref(*frame_config.get())));

  frame_configs_.push_back(std::move(frame_config));
}

void zivid_camera::ZividCamera::onCaptureGeneralConfigChanged(zivid_camera::CaptureGeneralConfig& config, uint32_t)
{
  ROS_INFO("%s", __func__);
  currentCaptureGeneralConfig_ = config;
}

void zivid_camera::ZividCamera::onCaptureFrameConfigChanged(zivid_camera::CaptureFrameConfig& config, uint32_t,
                                                            DRFrameConfig& frameConfig)
{
  ROS_INFO("%s name='%s'", __func__, frameConfig.name.c_str());
  frameConfig.config = config;
}

bool zivid_camera::ZividCamera::captureServiceHandler(zivid_camera::Capture::Request&, zivid_camera::Capture::Response&)
{
  ROS_DEBUG("%s", __func__);

  std::vector<Zivid::Settings> settings;

  Zivid::Settings baseSetting = camera_.settings();
  applyCaptureGeneralConfigToZividSettings(currentCaptureGeneralConfig_, baseSetting);

  for (const auto& frame_config : frame_configs_)
  {
    if (frame_config->config.enabled)
    {
      ROS_DEBUG("Config %s is enabled", frame_config->name.c_str());
      Zivid::Settings s{ baseSetting };
      applyCaptureFrameConfigToZividSettings(frame_config->config, s);
      settings.push_back(s);
    }
  }

  if (settings.size() > 0)
  {
    ROS_INFO("Capturing with %zd frames", settings.size());
    std::vector<Zivid::Frame> frames;
    frames.reserve(settings.size());
    for (const auto& s : settings)
    {
      camera_.setSettings(s);
      ROS_DEBUG("Calling capture() with settings: %s", camera_.settings().toString().c_str());
      frames.emplace_back(camera_.capture());
    }

    auto frame = [&]() {
      if (frames.size() > 1)
      {
        ROS_DEBUG("Calling Zivid::HDR::combineFrames with %zd frames", frames.size());
        return Zivid::HDR::combineFrames(begin(frames), end(frames));
      }
      else
      {
        return frames[0];
      }
    }();
    publishFrame(std::move(frame));
    return true;
  }
  else
  {
    ROS_ERROR("Capture called with 0 enabled frames!");
  }
  return false;
}

void zivid_camera::ZividCamera::publishFrame(Zivid::Frame&& frame)
{
  const bool hasPointCloudSubs = point_cloud_publisher_.getNumSubscribers() > 0;
  const bool hasRgbSubs = rgb_image_publisher_.getNumSubscribers() > 0;
  const bool hasDepthImgSubs = depth_image_publisher_.getNumSubscribers() > 0;

  if (hasPointCloudSubs || hasRgbSubs || hasDepthImgSubs)
  {
    auto point_cloud = frame.getPointCloud();

    std_msgs::Header header;
    header.seq = header_seq_++;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id_;

    if (hasPointCloudSubs)
    {
      ROS_INFO("Publishing point cloud");
      point_cloud_publisher_.publish(makePointCloud2(header, point_cloud));
    }

    if (hasRgbSubs)
    {
      ROS_INFO("Publishing rgb image");
      rgb_image_publisher_.publish(makeRgbImage(header, point_cloud));
    }

    if (hasDepthImgSubs)
    {
      ROS_INFO("Publishing depth image");
      depth_image_publisher_.publish(makeDepthImage(header, point_cloud));
    }
  }
}

sensor_msgs::PointCloud2 zivid_camera::ZividCamera::makePointCloud2(const std_msgs::Header& header,
                                                                    const Zivid::PointCloud& point_cloud)
{
  sensor_msgs::PointCloud2 msg;
  fillCommonMsgFields(msg, header, point_cloud);
  msg.point_step = sizeof(Zivid::Point);
  msg.row_step = msg.point_step * msg.width;
  msg.is_dense = false;

  msg.fields.reserve(5);
  msg.fields.push_back(createPointField("x", 0, 7, 1));
  msg.fields.push_back(createPointField("y", 4, 7, 1));
  msg.fields.push_back(createPointField("z", 8, 7, 1));
  msg.fields.push_back(createPointField("c", 12, 7, 1));
  msg.fields.push_back(createPointField("rgb", 16, 7, 1));

  msg.data =
      std::vector<uint8_t>((uint8_t*)point_cloud.dataPtr(), (uint8_t*)(point_cloud.dataPtr() + point_cloud.size()));

#pragma omp parallel for
  for (std::size_t i = 0; i < point_cloud.size(); i++)
  {
    uint8_t* point_ptr = &(msg.data[i * sizeof(Zivid::Point)]);
    float* x_ptr = (float*)&(point_ptr[msg.fields[0].offset]);
    float* y_ptr = (float*)&(point_ptr[msg.fields[1].offset]);
    float* z_ptr = (float*)&(point_ptr[msg.fields[2].offset]);

    // Convert from mm to m.
    *x_ptr *= 0.001f;
    *y_ptr *= 0.001f;
    *z_ptr *= 0.001f;
  }
  return msg;
}

sensor_msgs::Image zivid_camera::ZividCamera::makeRgbImage(const std_msgs::Header& header,
                                                           const Zivid::PointCloud& point_cloud)
{
  sensor_msgs::Image img;
  fillCommonMsgFields(img, header, point_cloud);
  img.encoding = sensor_msgs::image_encodings::RGB8;
  img.step = 3 * point_cloud.width();
  img.data.resize(img.step * img.height);

#pragma omp parallel for
  for (std::size_t i = 0; i < point_cloud.size(); i++)
  {
    img.data[3 * i + 0] = point_cloud(i).red();
    img.data[3 * i + 1] = point_cloud(i).green();
    img.data[3 * i + 2] = point_cloud(i).blue();
  }
  return img;
}

sensor_msgs::Image zivid_camera::ZividCamera::makeDepthImage(const std_msgs::Header& header,
                                                             const Zivid::PointCloud& point_cloud)
{
  sensor_msgs::Image img;
  fillCommonMsgFields(img, header, point_cloud);
  img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  img.step = 4 * point_cloud.width();
  img.data.resize(img.step * img.height);

#pragma omp parallel for
  for (std::size_t i = 0; i < point_cloud.size(); i++)
  {
    float* image_data = reinterpret_cast<float*>(&img.data[4 * i]);
    *image_data = point_cloud(i).z * 0.001;
  }
  return img;
}
