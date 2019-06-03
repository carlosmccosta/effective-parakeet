#include <ros/ros.h>

#include "zivid_camera.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zivid_camera");

  ros::NodeHandle nh;

  try
  {
    zivid_camera::ZividCamera wrapper(nh);
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error during initialization: %s", e.what());
    return EXIT_FAILURE;
  }

  return 0;
}
