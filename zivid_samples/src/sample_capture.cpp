#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <zivid_camera/CaptureFrameConfig.h>
#include <zivid_camera/Capture.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>

void onPointCloud(const sensor_msgs::PointCloud2ConstPtr&)
{
  ROS_INFO("PointCloud received");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_capture");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/zivid_camera/point_cloud", 10, onPointCloud);

  ROS_INFO("Updating camera settings");
  zivid_camera::CaptureFrameConfig frameCfg = zivid_camera::CaptureFrameConfig::__getDefault__();
  frameCfg.iris = 22;
  frameCfg.exposure_time = 0.02;

  dynamic_reconfigure::Reconfigure reconfig;
  frameCfg.__toMessage__(reconfig.request.config);
  ros::service::call("/zivid_camera/capture_frame/frame_0/set_parameters", reconfig);

  ros::Rate loop_rate(3);
  while (ros::ok())
  {
    ROS_INFO("Calling capture");
    zivid_camera::Capture capture;
    ros::service::call("/zivid_camera/capture", capture);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}