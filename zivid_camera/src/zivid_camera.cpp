#include "zivid_camera.hpp"
#include "parse_settings.hpp"
#include "zivid_camera/BooleanState.h"
#include "zivid_camera/FloatState.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/config_tools.h>

#include <Zivid/HDR.h>
#include <Zivid/Firmware.h>

#include <boost/algorithm/string.hpp>

#include <sstream>

namespace
{
template <typename ValueType>
struct ValueTypeToRosType
{
};
// TODO probably we want to use std_msg types
template <>
struct ValueTypeToRosType<bool>
{
  using type = zivid_camera::BooleanState;
};
template <>
struct ValueTypeToRosType<float>
{
  using type = zivid_camera::FloatState;
};
template <>
struct ValueTypeToRosType<double>
{
  using type = zivid_camera::FloatState;
};

template <typename ZividType>
ros::ServiceServer createCameraStateService(ros::NodeHandle& nh, const Zivid::Camera& camera)
{
  // TODO handle better unknown states, ignore or error?
  using RosType = typename ValueTypeToRosType<typename ZividType::ValueType>::type;

  std::string servicePath =
      std::string("camera_state/") + (boost::algorithm::to_lower_copy(std::string(ZividType::path)));

  ROS_INFO("Advertising cam state service for '%s' at '%s'", ZividType::name, servicePath.c_str());

  boost::function<bool(typename RosType::Request&, typename RosType::Response&)> callback =
      [&camera](typename RosType::Request&, typename RosType::Response& res) {
        res.value = camera.state().get<ZividType>().value();
        return true;
      };
  return nh.advertiseService(servicePath, callback);
}

template <class RootNode, class ListType>
auto setupCameraStateServices(const RootNode& rootNode, ListType& list, ros::NodeHandle& nh,
                              const Zivid::Camera& camera)
{
  rootNode.traverseValues([&](const auto& childNode) {
    using ChildType = std::remove_const_t<std::remove_reference_t<decltype(childNode)>>;
    list.push_back(createCameraStateService<ChildType>(nh, camera));
  });
}

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

}  // namespace

zivid_camera::ZividCamera::ZividCamera()
  : frame_id_(0)
  , priv_("~")
  , capture_general_dynreconfig_node_("~/capture_general")
  , capture_general_dynreconfig_server_(capture_general_dynreconfig_node_)
  , currentCaptureGeneralConfig_(zivid_camera::CaptureGeneralConfig::__getDefault__())
  , image_transport_(priv_)
{
  ROS_INFO("Zivid ROS driver version %s", ZIVID_ROS_DRIVER_VERSION);
  ROS_INFO("Built towards Zivid API version %s", ZIVID_VERSION);
  ROS_INFO("Running with Zivid API version %s", Zivid::Version::libraryVersion().c_str());
  if (Zivid::Version::libraryVersion() != ZIVID_VERSION)
  {
    throw std::string("Zivid library mismatch! The running Zivid version does not match the "
                      "version this library was built towards. Hint: Try to clean and re-build your project "
                      "from scratch.");
  }

  std::string serial_number;
  priv_.param<std::string>("serial_number", serial_number, "");

  std::string file_camera_path;
  priv_.param<std::string>("file_camera_path", file_camera_path, "");
  const bool file_camera_mode = !file_camera_path.empty();

  if (file_camera_mode)
  {
    ROS_INFO("Creating file camera from file '%s'", file_camera_path.c_str());
    camera_ = zivid_.createFileCamera(file_camera_path);
  }
  else
  {
    auto cameras = zivid_.cameras();
    if (cameras.empty())
    {
      throw std::runtime_error("No cameras found. Ensure that the camera is connected to the USB3 port on your PC.");
    }
    else if (serial_number.empty())
    {
      ROS_INFO("Selecting first camera");
      camera_ = cameras[0];
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

  ROS_INFO("Setting up reconfigurable params");
  // TODO be able to configure num frames
  for (int i = 0; i < 10; i++)
  {
    newSettings("capture_frame/frame_" + std::to_string(i));
  }

  capture_general_dynreconfig_server_.setCallback(
      boost::bind(&zivid_camera::ZividCamera::captureGeneralReconfigureCb, this, _1, _2));

  const auto pointCloudTopic = "point_cloud";
  ROS_INFO("Registering point cloud topic at '%s'", pointCloudTopic);
  pointcloud_pub_ = priv_.advertise<sensor_msgs::PointCloud2>(pointCloudTopic, 1);

  color_image_publisher_ = image_transport_.advertise("color/image_rect_color", 3);
  depth_image_publisher_ = image_transport_.advertise("depth/image", 3);

  const auto captureServiceName = "capture";
  ROS_INFO("Registering pointcloud capture service at '%s'", captureServiceName);
  boost::function<bool(zivid_camera::Capture::Request&, zivid_camera::Capture::Response&)> capture_callback_func =
      boost::bind(&zivid_camera::ZividCamera::captureServiceHandler, this, _1, _2);
  capture_service_ = priv_.advertiseService(captureServiceName, capture_callback_func);

  // setupCameraStateServices(camera_.state(), generated_servers_, priv_, camera_);

  const auto cameraInfoServiceName = "camera_info";
  ROS_INFO("Registering camera_info service at '%s'", cameraInfoServiceName);
  boost::function<bool(zivid_camera::CameraInfo::Request&, zivid_camera::CameraInfo::Response&)>
      zivid_info_callback_func = boost::bind(&zivid_camera::ZividCamera::cameraInfoServiceHandler, this, _1, _2);
  zivid_info_service_ = priv_.advertiseService(cameraInfoServiceName, zivid_info_callback_func);

  ROS_INFO("Zivid camera node is now ready!");
}

zivid_camera::ZividCamera::~ZividCamera() = default;

void zivid_camera::ZividCamera::newSettings(const std::string& name)
{
  frame_configs_.emplace_back();
  auto& frame_config = frame_configs_.back();

  frame_config.name = name;
  frame_config.reconfigure_server =
      std::make_shared<dynamic_reconfigure::Server<zivid_camera::CaptureFrameConfig>>(ros::NodeHandle("~/" + name));

  // Note that setting the callback below causes the server object to invoke the callback once
  // with the current configuration.
  frame_config.reconfigure_server->setCallback(
      boost::bind(&zivid_camera::ZividCamera::settingsReconfigureCallback, this, _1, _2, name));
}

void zivid_camera::ZividCamera::settingsReconfigureCallback(zivid_camera::CaptureFrameConfig& config, uint32_t,
                                                            const std::string& name)
{
  ROS_INFO("%s name='%s'", __func__, name.c_str());

  for (auto& frame_config : frame_configs_)
  {
    if (frame_config.name == name)
    {
      ROS_INFO("Frame config '%s' updated", frame_config.name.c_str());
      frame_config.config = config;
    }
  }
}

void zivid_camera::ZividCamera::captureGeneralReconfigureCb(zivid_camera::CaptureGeneralConfig& config, uint32_t)
{
  ROS_INFO("%s", __func__);
  currentCaptureGeneralConfig_ = config;
}

bool zivid_camera::ZividCamera::captureServiceHandler(zivid_camera::Capture::Request&, zivid_camera::Capture::Response&)
{
  ROS_INFO("Received capture request");

  std::vector<Zivid::Settings> settings;

  Zivid::Settings baseSetting = camera_.settings();
  // TODO how to handle different defaults for fex outlier filter.

  // apply currentCaptureGeneralConfig_.
  // TODO autogen this.
  baseSetting.set(Zivid::Settings::RedBalance{ currentCaptureGeneralConfig_.red_balance });
  baseSetting.set(Zivid::Settings::BlueBalance{ currentCaptureGeneralConfig_.blue_balance });
  baseSetting.set(
      Zivid::Settings::Filters::Reflection::Enabled{ currentCaptureGeneralConfig_.filters_reflection_enabled });
  baseSetting.set(
      Zivid::Settings::Filters::Saturated::Enabled{ currentCaptureGeneralConfig_.filters_saturated_enabled });
  baseSetting.set(Zivid::Settings::Filters::Gaussian::Enabled{ currentCaptureGeneralConfig_.filters_gaussian_enabled });
  baseSetting.set(Zivid::Settings::Filters::Gaussian::Sigma{ currentCaptureGeneralConfig_.filters_gaussian_sigma });
  baseSetting.set(Zivid::Settings::Filters::Contrast::Enabled{ currentCaptureGeneralConfig_.filters_contrast_enabled });
  baseSetting.set(
      Zivid::Settings::Filters::Contrast::Threshold{ currentCaptureGeneralConfig_.filters_contrast_threshold });
  baseSetting.set(Zivid::Settings::Filters::Outlier::Enabled{ currentCaptureGeneralConfig_.filters_outlier_enabled });
  baseSetting.set(
      Zivid::Settings::Filters::Outlier::Threshold{ currentCaptureGeneralConfig_.filters_outlier_threshold });

  for (const auto& frame_config : frame_configs_)
  {
    const auto& config = frame_config.config;

    if (config.enabled)
    {
      Zivid::Settings s{ baseSetting };
      // TODO autogen this.
      s.set(Zivid::Settings::Iris{ static_cast<std::size_t>(config.iris) });
      s.set(Zivid::Settings::ExposureTime{
          std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<double>(config.exposure_time)) });
      s.set(Zivid::Settings::Brightness{ config.brightness });
      s.set(Zivid::Settings::Gain{ config.gain });
      s.set(Zivid::Settings::Bidirectional{ config.bidirectional });
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
      ROS_DEBUG("Calling capture with settings: %s", camera_.settings().toString().c_str());
      frames.emplace_back(camera_.capture());
    }

    auto frame = [&]() {
      if (frames.size() > 1)
      {
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

bool zivid_camera::ZividCamera::cameraInfoServiceHandler(zivid_camera::CameraInfo::Request&,
                                                         zivid_camera::CameraInfo::Response& res)
{
  // TODO investigate if this should be individual services
  res.model_name = camera_.modelName();
  res.camera_revision = camera_.revision().toString();
  res.serial_number = camera_.serialNumber().toString();
  res.firmware_version = camera_.firmwareVersion();

  return true;
}

void zivid_camera::ZividCamera::publishFrame(Zivid::Frame&& frame)
{
  const bool hasPointCloudSubs = pointcloud_pub_.getNumSubscribers() > 0;
  const bool hasColorImgSubs = color_image_publisher_.getNumSubscribers() > 0;
  const bool hasDepthImgSubs = depth_image_publisher_.getNumSubscribers() > 0;

  if (hasPointCloudSubs || hasColorImgSubs || hasDepthImgSubs)
  {
    auto point_cloud = frame.getPointCloud();

    std_msgs::Header header;
    header.seq = frame_id_++;
    header.stamp = ros::Time::now();
    header.frame_id = "zivid_optical_frame";

    if (hasPointCloudSubs)
    {
      ROS_INFO("Publishing point cloud");
      pointcloud_pub_.publish(makePointCloud2(header, point_cloud));
    }

    if (hasColorImgSubs)
    {
      ROS_INFO("Publishing color image");
      color_image_publisher_.publish(makeColorImage(header, point_cloud));
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

sensor_msgs::Image zivid_camera::ZividCamera::makeColorImage(const std_msgs::Header& header,
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
