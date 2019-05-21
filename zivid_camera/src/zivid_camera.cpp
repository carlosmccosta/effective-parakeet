#include "zivid_camera.hpp"
#include "parse_settings.hpp"
#include "zivid_camera/BooleanState.h"
#include "zivid_camera/FloatState.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <dynamic_reconfigure/config_tools.h>

#include <Zivid/HDR.h>
#include <Zivid/Firmware.h>

#include <boost/algorithm/string.hpp>

#include <condition_variable>
#include <csignal>
#include <iostream>
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
}  // namespace

zivid_camera::ZividCamera::ZividCamera()
  : camera_mode_(0)
  , frame_id_(0)
  , camera_reconfigure_handler_("~/camera_config")
  , camera_reconfigure_server_(camera_reconfigure_handler_)
  , capture_general_dynreconfig_node_("~/capture_general_settings")
  , capture_general_dynreconfig_server_(capture_general_dynreconfig_node_)
  , currentCaptureGeneralConfig_(zivid_camera::CaptureGeneralSettingsConfig::__getDefault__())
{
  ROS_INFO("Zivid ROS driver version 0.0.1");  // todo get from cmake
  ROS_INFO("Built towards Zivid API version %s", ZIVID_VERSION);
  ROS_INFO("Running with Zivid API version %s", Zivid::Version::libraryVersion().c_str());

  ros::NodeHandle nh;
  ros::NodeHandle priv("~");

  // First setup necessary parameters
  std::string serial_number;
  priv.param<std::string>("serial_number", serial_number, "");

  bool test_mode_enabled;
  nh.param<bool>("zivid_test_mode_enabled", test_mode_enabled, false);
  std::string test_mode_file;
  if (test_mode_enabled)
  {
    nh.param<std::string>("zivid_test_mode_file", test_mode_file, "test.zdf");
    ROS_INFO("Using test file %s", test_mode_file.c_str());
  }

  if (!test_mode_enabled)
  {
    auto cameras = zivid_.cameras();
    if (cameras.empty())
    {
      throw std::runtime_error("No cameras found");
    }
    else if (serial_number.empty())
    {
      ROS_INFO("Selecting first camera");
      camera_ = cameras[0];
    }
    else
    {
      if (serial_number.find("sn") != 0)
      {
        throw std::runtime_error("Unrecognized serial number. The serial number must begin with 'sn'.");
      }

      const auto sn = serial_number.substr(2);
      camera_ = [&]() {
        ROS_INFO("Searching for camera with serial number %s ...", sn.c_str());
        for (auto& c : cameras)
        {
          if (c.serialNumber() == Zivid::SerialNumber(sn))
            return c;
        }
        throw std::runtime_error("No camera found with serial number " + sn);
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
  else
  {
    ROS_INFO("Test mode enabled");
    ROS_INFO("Creating file camera '%s'", test_mode_file.c_str());
    camera_ = zivid_.createFileCamera(test_mode_file);
  }

  ROS_INFO("%s", camera_.toString().c_str());
  ROS_INFO("Connecting to camera ...");
  camera_.connect();
  ROS_INFO("Connected to camera");

  camera_.setFrameCallback(boost::bind(&zivid_camera::ZividCamera::frameCallbackFunction, this, _1));

  ROS_INFO("Setting up reconfigurable params");
  // TODO dynamically grow/shrink this list.
  newSettings("frame_settings/frame_0");
  newSettings("frame_settings/frame_1");
  newSettings("frame_settings/frame_2");

  camera_reconfigure_server_.setCallback(
      boost::bind(&zivid_camera::ZividCamera::cameraReconfigureCallback, this, _1, _2));

  capture_general_dynreconfig_server_.setCallback(
      boost::bind(&zivid_camera::ZividCamera::captureGeneralReconfigureCb, this, _1, _2));

  ROS_INFO("Registering pointcloud topic at '%s'", "pointcloud");
  pointcloud_pub_ = priv.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

  ROS_INFO("Registering pointcloud capture service at '%s'", "capture");
  boost::function<bool(zivid_camera::Capture::Request&, zivid_camera::Capture::Response&)> capture_callback_func =
      boost::bind(&zivid_camera::ZividCamera::captureServiceHandler, this, _1, _2);
  capture_service_ = priv.advertiseService("capture", capture_callback_func);

  setupCameraStateServices(camera_.state(), generated_servers_, priv, camera_);

  ROS_INFO("Registering camera_info service at '%s'", "camera_info");
  boost::function<bool(zivid_camera::CameraInfo::Request&, zivid_camera::CameraInfo::Response&)>
      zivid_info_callback_func = boost::bind(&zivid_camera::ZividCamera::cameraInfoServiceHandler, this, _1, _2);
  zivid_info_service_ = priv.advertiseService("camera_info", zivid_info_callback_func);
}

zivid_camera::ZividCamera::~ZividCamera()
{
  ROS_INFO("~ZividCamera");
  if (camera_mode_ == ZividCamera_Live)
    camera_.stopLive();
}

sensor_msgs::PointCloud2 zivid_camera::ZividCamera::zividFrameToPointCloud2(const Zivid::Frame& frame)
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

void zivid_camera::ZividCamera::newSettings(const std::string& name)
{
  dynamic_reconfigure_settings_list_.emplace_back();
  auto& reconfigure_settings = dynamic_reconfigure_settings_list_.back();

  reconfigure_settings.name = name;
  reconfigure_settings.node_handle = ros::NodeHandle("~/" + name);
  reconfigure_settings.reconfigure_server =
      std::make_shared<dynamic_reconfigure::Server<zivid_camera::CaptureFrameSettingsConfig>>(
          reconfigure_settings.node_handle);
  reconfigure_settings.reconfigure_server->setCallback(
      boost::bind(&zivid_camera::ZividCamera::settingsReconfigureCallback, this, _1, _2, name));

  // This is necessary since the default constructed config object does not set the members
  // to their default values. See https://github.com/ros/dynamic_reconfigure/issues/33.
  reconfigure_settings.config = zivid_camera::CaptureFrameSettingsConfig::__getDefault__();
}

void zivid_camera::ZividCamera::frameCallbackFunction(const Zivid::Frame& frame)
{
  sensor_msgs::PointCloud2 pointcloud_msg = zividFrameToPointCloud2(frame);
  pointcloud_pub_.publish(pointcloud_msg);
}

void zivid_camera::ZividCamera::settingsReconfigureCallback(zivid_camera::CaptureFrameSettingsConfig& config, uint32_t,
                                                            const std::string& name)
{
  ROS_INFO("Dynamic reconfigure of node '%s'", name.c_str());

  for (auto& reconfigure_settings : dynamic_reconfigure_settings_list_)
  {
    if (reconfigure_settings.name == name)
    {
      ROS_INFO("You updated setting %s", reconfigure_settings.name.c_str());
      reconfigure_settings.config = config;
    }
  }
}

void zivid_camera::ZividCamera::cameraReconfigureCallback(zivid_camera::ZividCameraConfig& config, uint32_t)
{
  ROS_INFO("%s", __func__);
  configureCameraMode(config.camera_mode);
}

void zivid_camera::ZividCamera::captureGeneralReconfigureCb(zivid_camera::CaptureGeneralSettingsConfig& config,
                                                            uint32_t)
{
  ROS_INFO("%s", __func__);
  currentCaptureGeneralConfig_ = config;
}

void zivid_camera::ZividCamera::configureCameraMode(int camera_mode)
{
  // TODO consider removing live mode, or ensure that camera configuration is applied
  // before starting live.

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

bool zivid_camera::ZividCamera::captureServiceHandler(zivid_camera::Capture::Request&, zivid_camera::Capture::Response&)
{
  ROS_INFO("Received capture request");

  if (camera_mode_ == ZividCamera_Capture)
  {
    std::vector<Zivid::Settings> settings;

    Zivid::Settings baseSetting = camera_.settings();
    // TODO how to handle different defaults for fex outlier filter.

    // apply currentCaptureGeneralConfig_.
    // TODO autogen this.
    baseSetting.set(Zivid::Settings::RedBalance{ currentCaptureGeneralConfig_.red_balance });
    baseSetting.set(Zivid::Settings::BlueBalance{ currentCaptureGeneralConfig_.blue_balance });
    // +filters

    for (auto& reconfigure_settings : dynamic_reconfigure_settings_list_)
    {
      const auto& config = reconfigure_settings.config;

      ROS_INFO("Frame '%s' iris '%d'", reconfigure_settings.name.c_str(), config.iris);
      if (config.iris > 0)
      {
        Zivid::Settings s{ baseSetting };

        // TODO autogen this.
        s.set(Zivid::Settings::Iris{ static_cast<std::size_t>(config.iris) });
        s.set(Zivid::Settings::ExposureTime{ std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::duration<double>(config.exposure_time)) });
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
        ROS_INFO("Calling capture with settings: %s", camera_.settings().toString().c_str());
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
      pointcloud_pub_.publish(zividFrameToPointCloud2(std::move(frame)));
      return true;
    }
    else
    {
      ROS_ERROR("Capture called with 0 active frames!");
    }
    return false;
  }
  else
  {
    ROS_ERROR("Unable to capture because camera_mode is not Capture.");
    return false;
  }
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
