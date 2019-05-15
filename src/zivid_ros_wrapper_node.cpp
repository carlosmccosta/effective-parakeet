#include "ros/ros.h"

#include "zivid_ros_wrapper/zivid_ros_wrapper.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

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
    // TODO why are not exceptions in wrapper ctor caught?
    ROS_ERROR("Error during initialization: %s", e.what());
    return EXIT_FAILURE;
  }

  return 0;
}
