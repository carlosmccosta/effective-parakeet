#include "zivid_ros_wrapper/zivid_ros_wrapper.hpp"

#include <sensor_msgs/point_cloud2_iterator.h>

#include <dynamic_reconfigure/config_tools.h>

#include "zivid_ros_wrapper_gen.hpp"
#include <zivid_ros_wrapper/BooleanState.h>
#include <zivid_ros_wrapper/FloatState.h>

#include <Zivid/HDR.h>

#include <boost/algorithm/string.hpp>

#include <condition_variable>
#include <csignal>
#include <iostream>
#include <sstream>

namespace {

  template<typename ValueType> struct ValueTypeToRosType {};
  template<> struct ValueTypeToRosType<bool> { using type = zivid_ros_wrapper::BooleanState; };
  template<> struct ValueTypeToRosType<float> { using type = zivid_ros_wrapper::FloatState; };
  template<> struct ValueTypeToRosType<double> { using type = zivid_ros_wrapper::FloatState; };

  template <typename ZividType>
  ros::ServiceServer createCameraStateService(ros::NodeHandle & nh, const Zivid::Camera &camera)
  {
    // TODO handle better unknown states, ignore or error?
    using RosType = typename ValueTypeToRosType<typename ZividType::ValueType>::type;

    std::string servicePath = std::string("camera_state/") + (boost::algorithm::to_lower_copy(std::string(ZividType::path)));

    ROS_INFO("Advertising cam state service for '%s' at '%s'", ZividType::name, servicePath.c_str());

    boost::function<bool(typename RosType::Request&, typename RosType::Response&)>
      callback = [&camera](typename RosType::Request& req, typename RosType::Response& res)
      {
        res.value = camera.state().get<ZividType>().value();
        return true;
      };
    return nh.advertiseService(servicePath, callback);
  }

  template<class RootNode, class ListType> auto setupCameraStateServices(
    const RootNode & rootNode, ListType & list, ros::NodeHandle & nh, const Zivid::Camera &camera)
  {
    rootNode.traverseValues([&](const auto &childNode) {
      using ChildType = std::remove_const_t<std::remove_reference_t<decltype(childNode)>>;
        list.push_back(
          createCameraStateService<ChildType>(nh, camera));
    });
  };

  sensor_msgs::PointField createPointField(std::string name, uint32_t offset, uint8_t datatype, uint32_t count)
  {
    sensor_msgs::PointField point_field;
    point_field.name = name;
    point_field.offset = offset;
    point_field.datatype = datatype;
    point_field.count = count;
    return point_field;
  }
}

zivid_ros_wrapper::ZividRosWrapper::ZividRosWrapper()
  : camera_reconfigure_handler_("~/camera_config")
  , camera_reconfigure_server_(camera_reconfigure_handler_)
  , camera_mode_(0)
{
  ROS_INFO("Zivid ROS driver version 0.0.1"); //todo get from cmake
  ROS_INFO("Zivid API version %s", Zivid::Version::libraryVersion().c_str());

  ros::NodeHandle nh;
  ros::NodeHandle priv("~");

  // First setup necessary parameters
  std::string serial_number = "";
  priv.param<std::string>("serial_number", serial_number, "");

  bool test_mode_enabled;
  nh.param<bool>("zivid_test_mode_enabled", test_mode_enabled, false);
  std::string test_mode_file;
  if (test_mode_enabled)
  {
    nh.param<std::string>("zivid_test_mode_file", test_mode_file, "test.zdf");
    ROS_INFO("Using test file %s", test_mode_file.c_str());
  }

  if (zivid_.cameras().empty()) {
    ROS_ERROR("No Zivid cameras connected");
    throw std::runtime_error("No Zivid Cameras connected");
  }

  if (!test_mode_enabled)
  {
    if (serial_number == "")
    {
      ROS_INFO("Connecting to first available Zivid camera ...");
      camera_ = zivid_.connectCamera();
    }
    else if (serial_number.find("sn") == 0) {
      const auto sn = serial_number.substr(2);
      ROS_INFO("Connecting to Zivid camera with serial number %s ...", sn.c_str());
      camera_ = zivid_.connectCamera(Zivid::SerialNumber(sn));
    }
    else
      ROS_ERROR("The format of the serial number is not recognized. It should start with sn in ROS because of ROS "
                "param conversion problems");
  }
  else
  {
    ROS_INFO("Test mode enabled");
    ROS_INFO("Connecting to file camera '%s' ...", test_mode_file.c_str());
    camera_ = zivid_.createFileCamera(test_mode_file);
  }

  ROS_INFO("Connected to camera");
  ROS_INFO("%s", camera_.toString().c_str());
  camera_.setFrameCallback(boost::bind(&zivid_ros_wrapper::ZividRosWrapper::frameCallbackFunction, this, _1));

  ROS_INFO("Setting up reconfigurable params");
  newSettings("frame_settings_1");
  newSettings("frame_settings_2");
  newSettings("frame_settings_3");

  /*if (priv.hasParam("settings_names"))
  {
    std::string settings_names;
    priv.getParam("settings_names", settings_names);

    int start_pos = 0;
    int end_pos = 0;
    while (start_pos < settings_names.size())
    {
      end_pos = settings_names.find(";", start_pos);
      if (end_pos < 0)
        end_pos = settings_names.size();

      std::string settings_name = settings_names.substr(start_pos, (end_pos - start_pos));
      newSettings(settings_name);

      start_pos = end_pos + 1;
    }
  }*/

  camera_reconfigure_server_.setCallback(
      boost::bind(&zivid_ros_wrapper::ZividRosWrapper::cameraReconfigureCallback, this, _1, _2));

  ROS_INFO("Registering pointcloud topic at '%s'", "pointcloud");
  pointcloud_pub_ = priv.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

  ROS_INFO("Registering pointcloud capture service at '%s'", "capture");
  boost::function<bool(zivid_ros_wrapper::Capture::Request&, zivid_ros_wrapper::Capture::Response&)>
      capture_callback_func = boost::bind(&zivid_ros_wrapper::ZividRosWrapper::captureServiceHandler, this, _1, _2);
  capture_service_ = priv.advertiseService("capture", capture_callback_func);

  ROS_INFO("Registering pointcloud hdr capture service at '%s'", "hdr");
  boost::function<bool(zivid_ros_wrapper::HDR::Request&, zivid_ros_wrapper::HDR::Response&)> hdr_callback_func =
      boost::bind(&zivid_ros_wrapper::ZividRosWrapper::hdrCaptureServiceHandler, this, _1, _2);
  hdr_service_ = priv.advertiseService("hdr", hdr_callback_func);

  setupCameraStateServices(camera_.state(), generated_servers_, priv, camera_);

  ROS_INFO("Registering camera_info service at '%s'", "camera_info");
  boost::function<bool(zivid_ros_wrapper::CameraInfo::Request&, zivid_ros_wrapper::CameraInfo::Response&)>
      zivid_info_callback_func =
          boost::bind(&zivid_ros_wrapper::ZividRosWrapper::cameraInfoServiceHandler, this, _1, _2);
  zivid_info_service_ = priv.advertiseService("camera_info", zivid_info_callback_func);
}

zivid_ros_wrapper::ZividRosWrapper::~ZividRosWrapper()
{
  ROS_INFO("~ZividRosWrapper");
  if (camera_mode_ == ZividCamera_Live)
    camera_.stopLive();
}

sensor_msgs::PointCloud2 zivid_ros_wrapper::ZividRosWrapper::zividFrameToPointCloud2(const Zivid::Frame& frame)
{
  Zivid::PointCloud point_cloud = frame.getPointCloud();
  sensor_msgs::PointCloud2 pointcloud_msg;

  // Setup header values
  pointcloud_msg.header.seq = frame_id_++;
  pointcloud_msg.header.stamp = ros::Time::now();
  pointcloud_msg.header.frame_id = "zividsensor";

  // Now copy the point cloud information.
  pointcloud_msg.height = point_cloud.height();
  pointcloud_msg.width = point_cloud.width();
  pointcloud_msg.is_bigendian = false;
  pointcloud_msg.point_step = sizeof(Zivid::Point);
  pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width;
  pointcloud_msg.is_dense = false;

  pointcloud_msg.fields.push_back(createPointField("x", 8, 7, 1));
  pointcloud_msg.fields.push_back(createPointField("y", 0, 7, 1));
  pointcloud_msg.fields.push_back(createPointField("z", 4, 7, 1));
  pointcloud_msg.fields.push_back(createPointField("c", 12, 7, 1));
  pointcloud_msg.fields.push_back(createPointField("rgb", 16, 7, 1));

  pointcloud_msg.data =
      std::vector<uint8_t>((uint8_t*)point_cloud.dataPtr(), (uint8_t*)(point_cloud.dataPtr() + point_cloud.size()));

#pragma omp parallel for
  for (std::size_t i = 0; i < point_cloud.size(); i++)
  {
    uint8_t* point_ptr = &(pointcloud_msg.data[i * sizeof(Zivid::Point)]);
    float* x_ptr = (float*)&(point_ptr[pointcloud_msg.fields[0].offset]);
    float* y_ptr = (float*)&(point_ptr[pointcloud_msg.fields[1].offset]);
    float* z_ptr = (float*)&(point_ptr[pointcloud_msg.fields[2].offset]);

    // TODO this rotation should actually be fixed in tf, even though it probably is slightly slower.
    *x_ptr *= 0.001f;
    *z_ptr *= -0.001f;
    *y_ptr *= -0.001f;
  }

  return pointcloud_msg;
}

void zivid_ros_wrapper::ZividRosWrapper::newSettings(const std::string& name)
{
  dynamic_reconfigure_settings_list_.emplace_back();
  auto & reconfigure_settings = dynamic_reconfigure_settings_list_.back();

  reconfigure_settings.node_handle = ros::NodeHandle("~/" + name);
  reconfigure_settings.reconfigure_server =
      std::make_shared<dynamic_reconfigure::Server<zivid_ros_wrapper::ZividFrameSettingsConfig>>(reconfigure_settings.node_handle);
  reconfigure_settings.reconfigure_server->setCallback(
      boost::bind(&zivid_ros_wrapper::ZividRosWrapper::settingsReconfigureCallback, this, _1, _2, name));
  reconfigure_settings.name = name;
  reconfigure_settings.settings = Zivid::Settings{};
}

void zivid_ros_wrapper::ZividRosWrapper::frameCallbackFunction(const Zivid::Frame& frame)
{
  sensor_msgs::PointCloud2 pointcloud_msg = zividFrameToPointCloud2(frame);
  pointcloud_pub_.publish(pointcloud_msg);
}

void zivid_ros_wrapper::ZividRosWrapper::settingsReconfigureCallback(zivid_ros_wrapper::ZividFrameSettingsConfig& config,
                                                                     uint32_t /*level*/, const std::string& name)
{
  ROS_INFO("Dynamic reconfigure of node '%s'", name.c_str());

  for (int i = 0; i < dynamic_reconfigure_settings_list_.size(); i++)
  {
    auto& reconfigure_settings = dynamic_reconfigure_settings_list_[i];
    if (reconfigure_settings.name == name)
    {
      ROS_INFO("You updated setting %s", reconfigure_settings.name.c_str());

      dynamic_reconfigure::Config msg;
      config.__toMessage__(msg);

      Zivid::Settings& settings = reconfigure_settings.settings;
      settings.traverseValues([msg, &settings](auto& s) {
        using SettingType = std::remove_reference_t<decltype(s)>;
        std::string config_path = convertSettingsPathToConfigPath(s.path);
        settings.set(SettingType(getConfigValueFromString<typename SettingType::ValueType>(config_path, msg)));
      });
      if (reconfigure_settings.name == "frame_settings_1")
        camera_.setSettings(settings);
    }
  }
}

void zivid_ros_wrapper::ZividRosWrapper::cameraReconfigureCallback(zivid_ros_wrapper::ZividCameraConfig& config,
                                                                   uint32_t /*level*/)
{
  ROS_INFO("Running camera dynamic reconfiguration");
  configureCameraMode(config.camera_mode);
}

void zivid_ros_wrapper::ZividRosWrapper::configureCameraMode(int camera_mode)
{
  if (camera_mode != camera_mode_)
  {
    ROS_INFO("Changing camera mode from '%d' to '%d'", camera_mode_, camera_mode);

    if (camera_mode_ == ZividCamera_Live)
    {
      camera_.stopLive();
    }

    if (camera_mode == ZividCamera_Live)
    {
      camera_.startLive();
    }

    camera_mode_ = camera_mode;
  }
}

bool zivid_ros_wrapper::ZividRosWrapper::captureServiceHandler(zivid_ros_wrapper::Capture::Request& /* req */,
                                                               zivid_ros_wrapper::Capture::Response& res)
{
  ROS_INFO("Received capture request");

  if (camera_mode_ == ZividCamera_Capture)
  {
    Zivid::Frame frame = camera_.capture();
    res.pointcloud = zividFrameToPointCloud2(frame);
    return true;
  }
  else
  {
    ROS_ERROR("Unable to capture because camera_mode is not Capture.");
    return false;
  }
}

bool zivid_ros_wrapper::ZividRosWrapper::hdrCaptureServiceHandler(zivid_ros_wrapper::HDR::Request& req,
                                                                  zivid_ros_wrapper::HDR::Response& res)
{
  ROS_INFO("Received HDR capture request");

  if (camera_mode_ == ZividCamera_Capture)
  {
    Zivid::Settings default_settings = camera_.settings();
    std::vector<Zivid::Frame> hdr_frames;

    for (int i = 0; i < req.iris_settings.size(); i++)
    {
      camera_ << Zivid::Settings::Iris{ req.iris_settings[i] };
      hdr_frames.emplace_back(camera_.capture());
    }

    auto hdr_frame = Zivid::HDR::combineFrames(begin(hdr_frames), end(hdr_frames));
    res.pointcloud = zividFrameToPointCloud2(hdr_frame);;
    camera_.setSettings(default_settings);
    return true;
  }
  else
  {
    ROS_ERROR("Unable to capture HDR because camera_mode is not Capture.");
  }
  return false;
}

bool zivid_ros_wrapper::ZividRosWrapper::cameraInfoServiceHandler(zivid_ros_wrapper::CameraInfo::Request&,
                                                                 zivid_ros_wrapper::CameraInfo::Response& res)
{
  res.model_name = camera_.modelName();
  res.camera_revision = camera_.revision().toString();
  res.serial_number = camera_.serialNumber().toString();
  res.firmware_version = camera_.firmwareVersion();

  return true;
}
