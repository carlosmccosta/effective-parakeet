#include "ros/ros.h"

#include "zivid_ros_wrapper/zivid_ros_wrapper.hpp"

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zivid_ros_wrapper");

  try
  {
    zivid_ros_wrapper::ZividRosWrapper wrapper;
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error during initialization: %s", e.what());
    return EXIT_FAILURE;
  }

  ROS_INFO("THIS POINT REACHED");
  return 0;
}
