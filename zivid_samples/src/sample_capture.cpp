#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <zivid_camera/CaptureFrameConfig.h>
#include <zivid_camera/CaptureGeneralConfig.h>
#include <zivid_camera/Capture.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>

#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

namespace
{
void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

void onPointCloud(const sensor_msgs::PointCloud2ConstPtr&)
{
  ROS_INFO("PointCloud received");
  capture();
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_capture");
  ros::NodeHandle n;

  CHECK(ros::service::waitForService("/zivid_camera/capture", ros::Duration(15)));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber sub = n.subscribe("/zivid_camera/depth/points", 1, onPointCloud);

  ROS_INFO("Enable the reflection filter");
  dynamic_reconfigure::Client<zivid_camera::CaptureGeneralConfig> captureGeneralClient("/zivid_camera/"
                                                                                       "capture_general/");
  zivid_camera::CaptureGeneralConfig config;
  CHECK(captureGeneralClient.getCurrentConfiguration(config, ros::Duration(15)));
  config.filters_reflection_enabled = true;
  CHECK(captureGeneralClient.setConfiguration(config));

  ROS_INFO("Enable and configure the first frame");
  dynamic_reconfigure::Client<zivid_camera::CaptureFrameConfig> frame0Client("/zivid_camera/capture_frame/frame_0/");

  zivid_camera::CaptureFrameConfig frame0Cfg;
  CHECK(frame0Client.getDefaultConfiguration(frame0Cfg, ros::Duration(15)));

  frame0Cfg.enabled = true;
  frame0Cfg.iris = 22;
  frame0Cfg.exposure_time = 0.02;
  CHECK(frame0Client.setConfiguration(frame0Cfg));

  capture();

  ros::waitForShutdown();

  return 0;
}