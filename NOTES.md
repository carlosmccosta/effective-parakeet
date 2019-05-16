Notes

*** Pickit
Capture :
    rostopic pub /request std_msgs/Int32 1
Get Points:
    rostopic echo /zivid/points_3d (sensor_msgs/PointCloud2)
Configuration:
    Single dyn reconfigure node with name zivid_stream. 3 ET and 3 iris (iris 0=disabled)


** Photoneo

Available ROS services
For input and output parameters of each service please see coresponding service file in srv folder.

~/V2/is_acquiring
~/V2/is_connected
~/V2/set_coordination_space
~/V2/set_transformation
~/V2/start_acquisition
~/V2/stop_acquisition
~/connect_camera
~/disconnect_camera
~/get_device_list
~/get_frame
~/get_hardware_indentification
~/get_supported_capturing_modes
~/is_acquiring
~/is_connected
~/save_frame
~/set_parameters
~/start_acquisition
~/stop_acquisition
~/trigger_image


Available ROS topics

    /phoxi_camera/pointcloud (sensor_msgs/PointCloud2)
    Point cloud

    /phoxi_camera/confidence_map (sensor_msgs/Image)
    Confidence map

    /phoxi_camera/normal_map (sensor_msgs/Image)
    Normal map

    /phoxi_camera/texture (sensor_msgs/Image)
    Texture