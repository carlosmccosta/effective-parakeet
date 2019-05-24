# ROS wrapper for Zivid

[![Build Status](https://travis-ci.org/nedrebo/effective-parakeet.svg?branch=master)](https://travis-ci.org/nedrebo/effective-parakeet)

THIS WRAPPER IS WORK IN PROGRESS. IT WILL CHANGE BEFORE FINAL RELEASE.

CURRENT KNOWN CAVEATS/DEFECTS:
- It is only possible to capture 1 frame at a time (HDR is not supported yet).
- Only Ubuntu 18.04 with ROS Melodic has been tested.

This is the official ROS package for Zivid 3D cameras. This enables usage of Zivid cameras as
a node in ROS. Read more about Zivid at https://www.zivid.com/.

## Installation
Follow this step-by-step guide to install the Zivid ROS driver on your system.
This package supports Ubuntu 18.04 and ROS version Melodic Morenia.

### Prerequisites

#### ROS
Follow guide at http://wiki.ros.org/ROS/Installation to install ROS Melodic.

Also install catkin.

```
sudo apt-get install python-catkin-tools
```

#### Git
```
sudo apt-get install git
```

#### OpenCL
An OpenCL 1.2 compatible GPU and OpenCL drivers are required by the Zivid Core library.
Follow guide at https://help.zivid.com to install OpenCL.

#### Zivid Core Library
Download and install the Zivid Core debian package from [our webpage](https://www.zivid.com/downloads).

Optionally install the Zivid Tools and Zivid Studio packages as well. They are not required by the ROS
driver but can be useful for testing the camera and troubleshooting.

### Building Zivid ROS wrapper

#### Setting up the catkin workspace

If you have not setup your catkin workspace before, this needs to be done first.
Follow the guide at http://wiki.ros.org/catkin/Tutorials/create_a_workspace.

Clone the Zivid ROS project into the src/ directory.
```
cd ~/catkin_ws/src
git clone https://github.com/nedrebo/effective-parakeet.git
```

Install dependencies.
```
cd ~/catkin_ws
apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Finally, build the zivid_camera and zivid_sample packages.

```
catkin build
```

## Getting started

Connect the Zivid Camera to your USB3 port on your PC. You can use the ZividListCameras tool
available in the zivid-tools package to confirm that your system has been configured correctly, and
that the camera is discovered by your PC.

You can also open Zivid Studio and connect to the camera. Close Zivid Studio before continuing with
the rest of this guide. If the camera is not found, try our troubleshooting wiki at https://help.zivid.com.

Start roscore

```
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
roscore
```

In a new terminal window start the `zivid_camera` node

```
source ~/catkin_ws/devel/setup.bash
rosrun zivid_camera zivid_camera_node
```

Check the logger output from the node to confirm that it finds and connects
to the camera. Look for a log line containing "Zivid camera node is now ready!".

In a new terminal window start the `zivid_samples_capture` node.

```
source ~/catkin_ws/devel/setup.bash
rosrun zivid_samples zivid_samples_capture
```

The `zivid_samples_capture` node will first configure the capture settings of the camera and then
trigger captures repeatedly twice a second.

If everything is working so far, the camera will now capture approximately 3 times per second.
You can visualize the point cloud, color image and depth image using rviz.

```
rosrun rviz rviz
```

In rviz, go til File -> Open Config and navigate to zivid_camera/rviz/camera_view.rviz. NOTE: Due
to a bug in rviz you may have to open the configuration file twice.

While the camera is capturing you can adjust the capture settings using rqt_reconfigure.

```
rosrun rqt_reconfigure rqt_reconfigure
```

A more detailed description of the `zivid_camera` node follows below.

For example code in C++ and Python, see the Examples section.

## Services

The zivid_camera node provides the following services.

`/zivid_camera/capture`
> Invoke this service to trigger a capture. The capture settings are configured using
> dynamic_reconfigure, see the section "Dynamic parameters" below. When more than 1 frame is enabled
> then an HDR capture is performed. The resulting point cloud and depth/color images are published
> as ROS topics.

`/zivid_camera/camera_info`


## Published topics

The zivid_camera node publishes on the following ROS topics.

`/zivid_camera/point_cloud (sensor_msgs/PointCloud2)`
> Point cloud data. Each time a capture is invoked the resulting point cloud is published
> on this topic. The included point fields are x, y, z (in meters), c (contrast value),
> and r, g, b (colors).

`/zivid_camera/color/image_rect_color (sensor_msgs/Image)`
> RGB image. The image is encoded as "rgb8".

`/zivid_camera/depth/image (sensor_msgs/Image)`
> Depth image. Each pixel contains the z-value (in meters). The image is encoded as 32-bit float.

## Configuration of capture settings

The capture settings can be configured using [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure).

Run [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure) to view or change the settings using a GUI.

```
rosrun rqt_reconfigure rqt_reconfigure
```

The capture settings available in the `zivid_camera` node matches the settings in the Zivid API.
To become more familiar with the available settings in Zivid, run Zivid Studio or visit
the [API reference](http://www.zivid.com/software/api-documentation) of the `Settings` class.

**Currently only single capture is supported. HDR is not supported.**

## Launch Parameters

TODO

## Examples

In the zivid_samples package we have added several example nodes in C++ and Python that demonstrate
how to use the Zivid camera in ROS. These samples can be used as a starting point for your own project.

### Sample Capture


This sample shows how to configure the capture settings using dynamic_reconfigure, how to subscribe to the
`zivid_camera/point_cloud` topic, and then repeatedly invoking the `/zivid_camera/capture` service.

[C++](./zivid_samples/src/sample_capture.cpp)
```
cd ~/catkin_ws && source devel/setup.bash
rosrun zivid_samples zivid_samples_capture
```

[Python](./zivid_samples/scripts/sample_capture.py)
```
cd ~/catkin_ws && source devel/setup.bash
rosrun zivid_samples scripts/sample_capture.py
```

## FAQ

TODO
