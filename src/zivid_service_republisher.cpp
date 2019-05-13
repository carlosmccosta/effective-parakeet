#include "ros/ros.h"
#include "zivid_ros_wrapper/Capture.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zivid_service_republisher");

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<zivid_ros_wrapper::Capture>("zivid_ros_wrapper_node/capture");
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud_republish", 1);

    zivid_ros_wrapper::Capture srv;

    ros::Rate rate(0.5);
    while(ros::ok())
    {
        if(client.call(srv))
        {
            ROS_INFO("The call was successfull");
            ROS_INFO("Republishing...");
            pub.publish(srv.response.pointcloud);
        }
        else
        {
            ROS_ERROR("Failed to call service capture");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
