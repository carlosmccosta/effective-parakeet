#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <zivid_camera/CaptureFrameSettingsConfig.h>
#include <zivid_camera/Capture.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>

void onPointCloud(const sensor_msgs::PointCloud2ConstPtr& input)
{
  ROS_INFO("Got a pointcloud!!");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_capture");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/zivid_camera/pointcloud", 10, onPointCloud);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  ros::Rate loop_rate(1);

  int iris = 22;
  while (ros::ok())
  {
    std::cout << "Updating camera settings" << std::endl;
    zivid_camera::CaptureFrameSettingsConfig frameSettingsCfg =
        zivid_camera::CaptureFrameSettingsConfig::__getDefault__();
    frameSettingsCfg.iris = iris++;
    frameSettingsCfg.exposure_time = 0.100;

    dynamic_reconfigure::Config config;
    frameSettingsCfg.__toMessage__(config);

    dynamic_reconfigure::Reconfigure reconfig;
    reconfig.request.config = config;
    ros::service::call("/zivid_camera/frame_settings/frame_1/set_parameters", reconfig);

    std::cout << "Calling capture!" << std::endl;
    zivid_camera::Capture capture;
    ros::service::call("/zivid_camera/capture", capture);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}