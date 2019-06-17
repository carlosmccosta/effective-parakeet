#include <nodelet/loader.h>
#include <ros/ros.h>
#include <cstdlib>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zivid_camera");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  const auto nodelet_name = "zivid_camera/nodelet";
  ROS_INFO("Loading nodelet '%s'", nodelet_name);

  const bool loaded = nodelet.load(ros::this_node::getName(), nodelet_name, remap, nargv);
  if (!loaded)
  {
    ROS_FATAL("Failed to load nodelet '%s'!", nodelet_name);
    return EXIT_FAILURE;
  }

  ROS_INFO("Successfully loaded nodelet '%s'", nodelet_name);
  ros::spin();
  return EXIT_SUCCESS;
}