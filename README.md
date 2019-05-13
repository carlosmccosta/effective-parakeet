# ROS wrapper for Zivid

[![Build Status](https://travis-ci.org/nedrebo/effective-parakeet.svg?branch=master)](https://travis-ci.org/nedrebo/effective-parakeet)

This is an unofficial, unversioned, unsupported, early prototype.

https://www.zivid.com/

# Description
This project is a wrapper around the Zivid API. This enables usage of the Zivid camera as a ROS node.

## Installation
This guide will explain the required steps to run the Zivid ROS node.
### Prerequisites
- ROS is required. Follow guide at [TODO] to install ROS
- Git is required: Install with: [TODO]
- Some GPU stuff requirement

### Install the Zivid API library
- Download the Zivid install packages from the web: [TODO]
- Install the package using [TODO]

### Install ROS wrapper
- Download the ROS wrapper using git [TODO command]
- cd ~/catkin_ws
- Run catkin_make

## Using the node
Run the node
rosrun zivid_ros_wrapper zivid_ros_wrapper_node
This will start the camera in live mode and publish pointclouds on the /pointcloud topic.

## Using multiple cameras
[TODO]
