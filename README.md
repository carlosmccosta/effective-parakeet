# ROS wrapper for Zivid

[![Build Status](https://travis-ci.org/nedrebo/effective-parakeet.svg?branch=master)](https://travis-ci.org/nedrebo/effective-parakeet)

WORK IN PROGRESS

This is the official ROS package for Zivid 3D cameras. This enables usage of Zivid cameras as
a node in ROS. Read more about Zivid at https://www.zivid.com/.

## Installation
Follow this step-by-step guide to install the Zivid ROS driver on your system.

### Prerequisites
This package supports Ubuntu 18.04 and ROS version Melodic Morenia.

#### ROS
Follow guide at http://wiki.ros.org/ROS/Installation to install ROS.

#### Git
Git is required. Install with: [TODO]

#### OpenCL
The OpenCL drivers are required by the Zivid Core library. Follow guide at X to install OpenCL.

#### Zivid Core Library
- Download the Zivid Core debian packages from the web: https://www.zivid.com/downloads
- Optionally also install the Zivid Tools and Zivid Studio packages.
- Install the package using [TODO]

### Download and build the Zivid ROS wrapper
- cd ~/catkin_ws/src
- git clone ABCDE
- cd ~/catkin_ws
- catkin build

## Start the camera node
- rosrun zivid_camera zivid_camera_node

This will start the zivid_camera node.

### Launch parameters

These parameters can be provided to the zivid_camera node at startup to configure
the camera.

/zivid_camera/camera_info
/zivid_camera/camera_state/available
/zivid_camera/camera_state/connected
/zivid_camera/camera_state/live
/zivid_camera/camera_state/temperature/dmd
/zivid_camera/camera_state/temperature/general
/zivid_camera/camera_state/temperature/led
/zivid_camera/camera_state/temperature/lens
/zivid_camera/camera_state/temperature/pcb

### Services

-- /zivid_camera/capture
-- /zivid_camera/camera_info

### Topics

-- /zivid_camera/point_cloud
-- /zivid_camera/color_image

### Dynamic reconfigure

## Examples

### Starting rViz

### Configure the camera using rqt_reconfigure

### Python sample code

## Using multiple cameras

## FAQ

