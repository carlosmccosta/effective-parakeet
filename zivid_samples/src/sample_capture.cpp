#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <zivid_camera/CaptureFrameConfig.h>
#include <zivid_camera/Capture.h>
#include <dynamic_reconfigure/Reconfigure.h>

void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  ros::service::call("/zivid_camera/capture", capture);
}

void onPointCloud(const sensor_msgs::PointCloud2ConstPtr&)
{
  ROS_INFO("PointCloud received");
  capture();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_capture");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/zivid_camera/point_cloud", 10, onPointCloud);

  ROS_INFO("Enable and configure the first frame");
  zivid_camera::CaptureFrameConfig frameCfg = zivid_camera::CaptureFrameConfig::__getDefault__();
  frameCfg.enabled = true;
  frameCfg.iris = 22;
  frameCfg.exposure_time = 0.02;
  dynamic_reconfigure::Reconfigure reconfig;
  frameCfg.__toMessage__(reconfig.request.config);
  ros::service::call("/zivid_camera/capture_frame/frame_0/set_parameters", reconfig);

  capture();

  ros::spin();

  return 0;
}