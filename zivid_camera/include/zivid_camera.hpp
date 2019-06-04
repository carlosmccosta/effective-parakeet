#ifndef ZIVID_CAMERA_H
#define ZIVID_CAMERA_H

#include <ros/ros.h>

#include <zivid_camera/CaptureFrameConfig.h>
#include <zivid_camera/CaptureGeneralConfig.h>
#include <zivid_camera/Capture.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>

#include <dynamic_reconfigure/server.h>

#include <Zivid/Application.h>
#include <Zivid/Camera.h>
#include <Zivid/Settings.h>
#include <Zivid/Version.h>

namespace zivid_camera
{
class ZividCamera
{
public:
  ZividCamera(ros::NodeHandle& nh);
  ~ZividCamera();

private:
  struct DRFrameConfig
  {
    DRFrameConfig(const std::string& name, ros::NodeHandle& nh)
      : name(name), dr_server(dr_server_mutex, ros::NodeHandle(nh, name)), config(CfgType::__getDefault__())
    {
    }
    using CfgType = zivid_camera::CaptureFrameConfig;
    std::string name;
    boost::recursive_mutex dr_server_mutex;
    dynamic_reconfigure::Server<CfgType> dr_server;
    CfgType config;
  };

  void setupCaptureGeneralConfigNode(const Zivid::Settings& cameraSettings);
  void setupCaptureFrameConfigNode(int nodeIdx, const Zivid::Settings& cameraSettings);
  void onCaptureGeneralConfigChanged(zivid_camera::CaptureGeneralConfig& config, uint32_t level);
  void onCaptureFrameConfigChanged(zivid_camera::CaptureFrameConfig& config, uint32_t level,
                                   DRFrameConfig& frameConfig);
  bool captureServiceHandler(zivid_camera::Capture::Request& req, zivid_camera::Capture::Response& res);
  void publishFrame(Zivid::Frame&& frame);
  sensor_msgs::PointCloud2 makePointCloud2(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud);
  sensor_msgs::Image makeRgbImage(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud);
  sensor_msgs::Image makeDepthImage(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud);

  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  boost::recursive_mutex capture_general_dr_server_mutex_;
  std::unique_ptr<dynamic_reconfigure::Server<zivid_camera::CaptureGeneralConfig>> capture_general_dr_server_;
  zivid_camera::CaptureGeneralConfig currentCaptureGeneralConfig_;
  ros::Publisher point_cloud_publisher_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher rgb_image_publisher_;
  image_transport::Publisher depth_image_publisher_;
  ros::ServiceServer capture_service_;
  ros::ServiceServer camera_info_serial_number_service_;
  ros::ServiceServer camera_info_model_name_service_;
  std::vector<std::unique_ptr<DRFrameConfig>> frame_configs_;
  Zivid::Application zivid_;
  Zivid::Camera camera_;
  std::string frame_id_;
  int header_seq_;
};
}  // namespace zivid_camera

#endif
