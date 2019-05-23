#include <ros/ros.h>

#include "zivid_camera.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zivid_camera");

  // TODO this nodehandle is created such that the node outlives the wrapper object, so that
  // exceptions are logged in the catch below. A handle is also created in wrapper. Decide
  // if / how to solve, maybe pass as ref into the class.
  ros::NodeHandle nh;

  try
  {
    zivid_camera::ZividCamera wrapper;
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error during initialization: %s", e.what());
    return EXIT_FAILURE;
  }

  return 0;
}
