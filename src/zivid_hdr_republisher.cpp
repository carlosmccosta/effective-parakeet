#include "ros/ros.h"
#include "zivid_ros_wrapper/HDR.h"
#include <zivid_ros_wrapper/TripleHDRConfig.h>

#include <dynamic_reconfigure/server.h>
#include "sensor_msgs/PointCloud2.h"
#include <iostream>
void reconfigure_handler(zivid_ros_wrapper::TripleHDRConfig &config, uint32_t /*level*/, zivid_ros_wrapper::HDR* hdr_srv)
{
    hdr_srv->request.iris_settings[0] = config.iris_1;
    hdr_srv->request.iris_settings[1] = config.iris_2;
    hdr_srv->request.iris_settings[2] = config.iris_3;
}

void setupHDRServiceMessage(zivid_ros_wrapper::HDR& srv)
{
    srv.request.iris_settings.push_back(13);
    srv.request.iris_settings.push_back(22);
    srv.request.iris_settings.push_back(33);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zivid_hdr_republisher");

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<zivid_ros_wrapper::HDR>("zivid_ros_wrapper_node/hdr");
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("hdr_pointcloud", 1);


    zivid_ros_wrapper::HDR srv;
    setupHDRServiceMessage(srv);

    dynamic_reconfigure::Server<zivid_ros_wrapper::TripleHDRConfig> camera_reconfigure_server_m;
    dynamic_reconfigure::Server<zivid_ros_wrapper::TripleHDRConfig>::CallbackType f;
    f = boost::bind(reconfigure_handler, _1, _2, &srv);
    camera_reconfigure_server_m.setCallback(f);

    ros::Rate rate(0.5);
    while(ros::ok())
    {
        if(client.call(srv))
        {
            ROS_INFO("Republishing...");
            pub.publish(srv.response.pointcloud);
        }
        else
        {
            ROS_ERROR("Failed to call service hdr. (Please start the zivid_ros_wrapper_node or wait about 20 seconds for it to start)");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
