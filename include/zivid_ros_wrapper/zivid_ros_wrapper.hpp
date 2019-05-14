#ifndef ZIVID_ROS_WRAPPER_H
#define ZIVID_ROS_WRAPPER_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <dynamic_reconfigure/server.h>
#include <zivid_ros_wrapper/ZividSettingsConfig.h>
#include <zivid_ros_wrapper/ZividCameraConfig.h>
#include <zivid_ros_wrapper/Capture.h>
#include <zivid_ros_wrapper/HDR.h>
#include <zivid_ros_wrapper/ZividInfo.h>

#include <Zivid/Zivid.h>

namespace zivid_ros_wrapper
{
    class ZividRosWrapper
    {
    public:
        ZividRosWrapper();
        ~ZividRosWrapper();
        void init();
        void disconnect();

    private:
        struct DynamicReconfigureSettings
        {
            ros::NodeHandle node_handle;
            std::shared_ptr<dynamic_reconfigure::Server<zivid_ros_wrapper::ZividSettingsConfig>> reconfigure_server;
            std::string name;
            Zivid::Settings settings;
        };

        Zivid::Point zividFrameToROSFrame(const Zivid::Point& point);
        sensor_msgs::PointCloud2 zividFrameToPointCloud2(const Zivid::Frame& frame);
        void frameCallbackFunction(const Zivid::Frame& frame);
        void settingsReconfigureCallback(zivid_ros_wrapper::ZividSettingsConfig &config, uint32_t level, const std::string& name);
        void newSettings(const std::string& name);
        void removeSettings(const std::string& name);
        void cameraReconfigureCallback(zivid_ros_wrapper::ZividCameraConfig &config, uint32_t level);
        void configureCameraMode(int camera_mode);
        bool captureServiceHandler(zivid_ros_wrapper::Capture::Request& req, zivid_ros_wrapper::Capture::Response& res);
        bool hdrCaptureServiceHandler(zivid_ros_wrapper::HDR::Request& req, zivid_ros_wrapper::HDR::Response& res);
        bool zividInfoServiceHandler(zivid_ros_wrapper::ZividInfo::Request& req, zivid_ros_wrapper::ZividInfo::Response& res);

        Zivid::Application zivid_;
        Zivid::Camera camera_;
        boost::function<void(const Zivid::Frame&)> local_function_link_m;
        int camera_mode_;
        ros::Publisher pointcloud_pub_m;
        ros::ServiceServer capture_service_m;
        ros::ServiceServer hdr_service_m;
        ros::ServiceServer new_settings_service_m;
        ros::ServiceServer copy_settings_service_m;
        ros::ServiceServer remove_settings_service_m;
        std::vector<ros::ServiceServer> generated_servers_m;
        ros::ServiceServer zivid_info_service_m;

        ros::NodeHandle camera_reconfigure_handler_m;
        dynamic_reconfigure::Server<zivid_ros_wrapper::ZividCameraConfig> camera_reconfigure_server_m;

        std::vector<DynamicReconfigureSettings> dynamic_reconfigure_settings_list_m;

        int frame_id_m;
    };
}

#endif
