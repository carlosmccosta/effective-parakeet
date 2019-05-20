#ifndef ZIVID_CAMERA_H
#define ZIVID_CAMERA_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <dynamic_reconfigure/server.h>
#include <zivid_camera/CaptureFrameSettingsConfig.h>
#include <zivid_camera/ZividCameraConfig.h>
#include <zivid_camera/CaptureGeneralSettingsConfig.h>
#include <zivid_camera/Capture.h>
#include <zivid_camera/HDR.h>
#include <zivid_camera/CameraInfo.h>

#include <Zivid/Application.h>
#include <Zivid/Camera.h>
#include <Zivid/Settings.h>
#include <Zivid/Version.h>

namespace zivid_camera
{
class ZividCamera
{
public:
  ZividCamera();
  ~ZividCamera();

private:
  struct DynamicReconfigureSettings
  {
    std::string name;
    ros::NodeHandle node_handle;
    std::shared_ptr<dynamic_reconfigure::Server<zivid_camera::CaptureFrameSettingsConfig>> reconfigure_server;
    //Zivid::Settings settings;
    zivid_camera::CaptureFrameSettingsConfig config;
  };

  Zivid::Point zividFrameToROSFrame(const Zivid::Point& point);
  sensor_msgs::PointCloud2 zividFrameToPointCloud2(const Zivid::Frame& frame);
  void frameCallbackFunction(const Zivid::Frame& frame);
  void settingsReconfigureCallback(zivid_camera::CaptureFrameSettingsConfig& config, uint32_t level,
                                   const std::string& name);
  void newSettings(const std::string& name);
  void cameraReconfigureCallback(zivid_camera::ZividCameraConfig& config, uint32_t level);
  void captureGeneralReconfigureCb(zivid_camera::CaptureGeneralSettingsConfig& config, uint32_t level);
  void configureCameraMode(int camera_mode);
  bool captureServiceHandler(zivid_camera::Capture::Request& req, zivid_camera::Capture::Response& res);
  bool cameraInfoServiceHandler(zivid_camera::CameraInfo::Request& req, zivid_camera::CameraInfo::Response& res);

  Zivid::Application zivid_;
  Zivid::Camera camera_;
  int camera_mode_;
  zivid_camera::CaptureGeneralSettingsConfig currentCaptureGeneralConfig_;

  ros::Publisher pointcloud_pub_;
  ros::ServiceServer capture_service_;
  std::vector<ros::ServiceServer> generated_servers_;
  ros::ServiceServer zivid_info_service_;
  ros::NodeHandle camera_reconfigure_handler_;
  dynamic_reconfigure::Server<zivid_camera::ZividCameraConfig> camera_reconfigure_server_;
  ros::NodeHandle capture_general_dynreconfig_node_;
  dynamic_reconfigure::Server<zivid_camera::CaptureGeneralSettingsConfig> capture_general_dynreconfig_server_;
  std::vector<DynamicReconfigureSettings> dynamic_reconfigure_settings_list_;
  int frame_id_;
};
}  // namespace zivid_camera

#endif
