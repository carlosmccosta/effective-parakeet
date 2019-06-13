#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zivid_camera");

  ROS_INFO("Creating nodelet::Loader");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  const auto nodelet_name = "zivid_camera/nodelet";
  ROS_INFO("Loading nodelet %s", nodelet_name);
  nodelet.load(ros::this_node::getName(), nodelet_name, remap, nargv);
  ros::spin();
  return 0;
}