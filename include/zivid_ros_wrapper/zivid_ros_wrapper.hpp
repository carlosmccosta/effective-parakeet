#ifndef ZIVID_ROS_WRAPPER_H
#define ZIVID_ROS_WRAPPER_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <dynamic_reconfigure/server.h>
#include <zivid_ros_wrapper/CaptureFrameSettingsConfig.h>
#include <zivid_ros_wrapper/ZividCameraConfig.h>
#include <zivid_ros_wrapper/CaptureGeneralSettingsConfig.h>
#include <zivid_ros_wrapper/Capture.h>
#include <zivid_ros_wrapper/HDR.h>
#include <zivid_ros_wrapper/CameraInfo.h>

#include <Zivid/Application.h>
#include <Zivid/Camera.h>
#include <Zivid/Settings.h>
#include <Zivid/Version.h>

namespace zivid_ros_wrapper
{
class ZividRosWrapper
{
public:
  ZividRosWrapper();
  ~ZividRosWrapper();

private:
  struct DynamicReconfigureSettings
  {
    ros::NodeHandle node_handle;
    std::shared_ptr<dynamic_reconfigure::Server<zivid_ros_wrapper::CaptureFrameSettingsConfig>> reconfigure_server;
    std::string name;
    Zivid::Settings settings;
  };

  Zivid::Point zividFrameToROSFrame(const Zivid::Point& point);
  sensor_msgs::PointCloud2 zividFrameToPointCloud2(const Zivid::Frame& frame);
  void frameCallbackFunction(const Zivid::Frame& frame);
  void settingsReconfigureCallback(zivid_ros_wrapper::CaptureFrameSettingsConfig& config, uint32_t level,
                                   const std::string& name);
  void newSettings(const std::string& name);
  void cameraReconfigureCallback(zivid_ros_wrapper::ZividCameraConfig& config, uint32_t level);
  void captureGeneralReconfigureCb(zivid_ros_wrapper::CaptureGeneralSettingsConfig& config, uint32_t level);
  void configureCameraMode(int camera_mode);
  bool captureServiceHandler(zivid_ros_wrapper::Capture::Request& req, zivid_ros_wrapper::Capture::Response& res);
  bool hdrCaptureServiceHandler(zivid_ros_wrapper::HDR::Request& req, zivid_ros_wrapper::HDR::Response& res);
  bool cameraInfoServiceHandler(zivid_ros_wrapper::CameraInfo::Request& req, zivid_ros_wrapper::CameraInfo::Response& res);

  Zivid::Application zivid_;
  Zivid::Camera camera_;
  int camera_mode_;
  ros::Publisher pointcloud_pub_;
  ros::ServiceServer capture_service_;
  ros::ServiceServer hdr_service_;
  std::vector<ros::ServiceServer> generated_servers_;
  ros::ServiceServer zivid_info_service_;

  ros::NodeHandle camera_reconfigure_handler_;
  dynamic_reconfigure::Server<zivid_ros_wrapper::ZividCameraConfig> camera_reconfigure_server_;

  ros::NodeHandle capture_general_dynreconfig_node_;
  dynamic_reconfigure::Server<zivid_ros_wrapper::CaptureGeneralSettingsConfig> capture_general_dynreconfig_server_;




  std::vector<DynamicReconfigureSettings> dynamic_reconfigure_settings_list_;

  int frame_id_;
};
}  // namespace zivid_ros_wrapper

#endif
