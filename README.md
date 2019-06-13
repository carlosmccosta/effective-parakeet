# ROS driver for Zivid

[![Build Status](https://travis-ci.org/nedrebo/effective-parakeet.svg?branch=master)](https://travis-ci.org/nedrebo/effective-parakeet)

THIS DRIVER IS WORK IN PROGRESS. IT WILL CHANGE BEFORE FINAL RELEASE.

This is the official ROS package for Zivid 3D cameras. This enables usage of Zivid cameras as
a node/nodelet in ROS. Read more about Zivid at [our website](https://www.zivid.com/).

<p align="center">
    <img src="https://www.zivid.com/software/zivid-ros/ros_zivid_camera.png" width="600" height="273">
</p>

## Installation
Follow this step-by-step guide to install the Zivid ROS driver on your system.
This package supports Ubuntu 18.04 and ROS Melodic Morenia.

### Prerequisites

#### ROS
Follow the [ROS installation wiki](http://wiki.ros.org/ROS/Installation) to install ROS Melodic.

Also install catkin and git.

```
sudo apt update
sudo apt install python-catkin-tools git
```

#### OpenCL
An OpenCL 1.2 compatible GPU and OpenCL drivers are required by the Zivid Core library.
Follow the guide at https://help.zivid.com to install OpenCL (search for Install OpenCL).

#### Zivid Core Library
Download and install the "Toshiba Teli driver" and "Zivid Core" debian packages from
[our webpage](https://www.zivid.com/downloads). Version 1.3 or newer is required.

Optionally install the "Zivid Studio" and "Zivid Tools" packages as well. They are not required by the ROS
driver but can be useful for testing that your system has been setup correctly and that
the camera is detected.

#### Create catkin workspace

If you have not created a catkin workspace, this needs to be done as well.
```
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin build
```

### Building Zivid ROS driver

Clone the Zivid ROS project into the ~/catkin_ws/src directory.
```
cd ~/catkin_ws/src
git clone https://github.com/nedrebo/effective-parakeet.git
```

Install dependencies.
```
cd ~/catkin_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Finally, build the `zivid_camera` and `zivid_samples` packages.

```
catkin build
```

## Getting started

Connect the Zivid Camera to your USB3 port on your PC. You can use the ZividListCameras tool
available in the zivid-tools package to confirm that your system has been configured correctly, and
that the camera is discovered by your PC. You can also open Zivid Studio and connect to the camera.
Close Zivid Studio before continuing with the rest of this guide. If the camera is not found, visit
our [troubleshooting wiki](https://help.zivid.com).

First, start `roscore`.

```
cd ~/catkin_ws && source devel/setup.bash
roscore
```

In a new terminal window start the `zivid_camera` node.

```
cd ~/catkin_ws && source devel/setup.bash
ROS_NAMESPACE=zivid_camera rosrun zivid_camera zivid_camera_node
```

Check the output from the node to confirm that it finds and connects
to the camera. Look for a log line containing "Zivid camera node is now ready!".

In a new terminal window start the `zivid_samples_capture` node.

```
cd ~/catkin_ws && source devel/setup.bash
rosrun zivid_samples zivid_samples_capture
```

The `zivid_samples_capture` node will first configure the capture settings of the camera and then
capture repeatedly. You can visualize the point cloud, color image and depth image using [rviz](https://wiki.ros.org/rviz).

```
cd ~/catkin_ws && source devel/setup.bash
roslaunch zivid_camera visualize.launch
```

You can adjust the capture settings using [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure).

```
cd ~/catkin_ws && source devel/setup.bash
rosrun rqt_reconfigure rqt_reconfigure
```

Try to adjust the exposure time or the iris and observe that the images and point cloud in
[rviz](https://wiki.ros.org/rviz) changes.

You can also use the launch script `sample_capture.launch` which does all of these
steps for you, including starting [rviz](https://wiki.ros.org/rviz) and
[rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure).

```
cd ~/catkin_ws && source devel/setup.bash
roslaunch zivid_samples sample_capture.launch
```

A more detailed description of the `zivid_camera` node follows below.

For sample code in C++ and Python, see the Samples section.

## Services

`capture` ([zivid_camera/Capture](./zivid_camera/srv/Capture.srv))
> Invoke this service to trigger a capture. The capture settings are configured using
> [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure), see the section "Dynamic
> parameters" below. When more than 1 frame is enabled an HDR capture is performed.
> The resulting point cloud and depth/color images are published as ROS topics.

`camera_info/model_name` ([zivid_camera/CameraInfoModelName](./zivid_camera/srv/CameraInfoModelName.srv))
> Returns the camera's model name.

`camera_info/serial_number` ([zivid_camera/CameraInfoSerialNumber](./zivid_camera/srv/CameraInfoSerialNumber.srv))
> Returns the camera's serial number.

## Topics
`color/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
> Camera calibration and metadata.

`color/image_color` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
> RGB image. The image is encoded as "rgb8".

`depth/points` ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))
> Point cloud data. Each time a capture is invoked the resulting point cloud is published
> on this topic. The included point fields are x, y, z (in meters), c (contrast value),
> and r, g, b (colors). The output is in the camera's optical frame, where x is right, y is
> down and z is forward.

`depth/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
> Camera calibration and metadata.

`depth/image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
> Depth image. Each pixel contains the z-value (along the camera Z axis) in meters.
> The image is encoded as 32-bit float. Pixels where z-value is missing are NaN.

## Configuration

The capture settings can be configured using [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure).
Use [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure) to view or change the settings using a GUI.

```
rosrun rqt_reconfigure rqt_reconfigure
```

The capture settings available in the `zivid_camera` node matches the settings in the Zivid API.
To become more familiar with the available settings in Zivid, run Zivid Studio or see
the [API reference](http://www.zivid.com/software/api-documentation) of the `Settings` class.
Note that the available settings will depend on which version of Zivid Core you are using.

The `zivid_camera` node supports both single-capture and HDR-capture. HDR-capture works by taking
several captures with different settings (for example different exposure time) and combining the
captures into one high-quality point cloud. For more information about HDR capture, visit our
[knowledge base](https://help.zivid.com) and search for HDR.

The available capture settings are split into subtrees:

```
/capture_frame
    /frame_0
        ...
    /frame_1
        ...
    ...
    /frame_9
        ...
/capture_general
    ...
```

`capture_general` contains common settings for all frames (as of Core version 1.3 this is all the
filters and color balance). `frame_settings/frame_<n>/` contains settings for an individual
frame. By default `<n>` can be 0 to 9 for a total of 10 configured frames. The total number of frames
can be configured using the launch parameter `num_capture_frames` (see below).

`frame_settings/frame_<n>/enabled` controls if the frame `<n>` is enabled. When the
`capture/` service is invoked, if one frame is enabled, the camera will perform a
single-capture. If more than one frame is enabled, the camera will perform an HDR capture.

By default all frames are disabled. In order to capture a point cloud at least one frame needs to be
enabled.

Note that the min, max and default value of the dynamic_reconfigure parameters can change dependent on
what Zivid camera model you are using. Therefore you should not use the `__getMin()__`, `__getMax()__` and
`__getDefault()__` methods of the auto-generated C++ config classes (`zivid_camera::CaptureGeneralConfig`
and `zivid_camera::CaptureFrameConfig`). Instead you should query the server for the default values.
See the sample code for how to do this.

### List of dynamic reconfigure parameters

#### General settings

`capture_general/*`
> The settings here applies to all the frames. Contains filters and color balance.

TODO: extend documentation.

#### Per-frame settings

`frame_settings/frame_<n>/enabled` (bool)
> Controls if the frame `<n>` is enabled. When the frame is enabled it will be included
> in captures. The default value is false.

`frame_settings/frame_<n>/bidirectional` (bool)
> Corresponds to the API setting [Zivid::Settings::Bidirectional](https://www.zivid.com/hubfs/softwarefiles/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Bidirectional.html).

`frame_settings/frame_<n>/brightness` (double)
> Corresponds to the API setting [Zivid::Settings::Brightness](https://www.zivid.com/hubfs/softwarefiles/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Brightness.html).

`frame_settings/frame_<n>/exposure_time` (double)
> Corresponds to the API setting [Zivid::Settings::ExposureTime](https://www.zivid.com/hubfs/softwarefiles/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1ExposureTime.html). Per ROS convention it is specified in seconds instead of milliseconds.

`frame_settings/frame_<n>/gain` (double)
> Corresponds to the API setting [Zivid::Settings::Gain](https://www.zivid.com/hubfs/softwarefiles/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Gain.html).

`frame_settings/frame_<n>/iris` (int)
> Corresponds to the API setting [Zivid::Settings::Iris](https://www.zivid.com/hubfs/softwarefiles/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Iris.html).

## Launch Parameters

The following parameters can be specified when starting the `zivid_camera` node.

`file_camera_path` (string, default: "")
> Specify the path to a file camera to use instead of a real Zivid camera. This can be used to
> develop without access to hardware. The file camera returns the same point cloud for every capture.
> [Click here to download a file camera.](https://www.zivid.com/software/ZividSampleData.zip)

`frame_id` (string, default: "zivid_optical_frame")
> Specify the frame_id used for all published images and point clouds.

`num_capture_frames` (int, default: 10)
> Specify the number of dynamic_reconfigure capture_frame nodes that are created during startup of
> the node. This number defines the maximum number of frames in a capture. If you need to perform
> HDR with more than 10 frames then increase this number.

`serial_number` (string, default: "")
> Specify the serial number of the Zivid camera to use. Important: When passing this value via
> the command line or rosparam the serial number must be prefixed with a colon (`:012345`).
> This parameter is optional. By default the driver will connect to the first available camera.

## Samples

In the zivid_samples package we have added an example node in C++ and Python that demonstrate
how to use the Zivid camera in ROS. The samples can be used as a starting point for your own project.

### Sample Capture

This sample performs a single-capture repeatedly. This sample shows how to configure the capture
settings using [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure), how to subscribe to
the `depth/points` topic, and how to invoke the `capture` service.

**C++** [(Source code)](./zivid_samples/src/sample_capture.cpp)
```
cd ~/catkin_ws && source devel/setup.bash
rosrun zivid_samples zivid_samples_capture
```

**Python** [(Source code)](./zivid_samples/scripts/sample_capture.py)
```
cd ~/catkin_ws && source devel/setup.bash
rosrun zivid_samples sample_capture.py
```

## Nodelet

zivid_camera can also be launched as a [nodelet](http://wiki.ros.org/nodelet), for example:

```
ROS_NAMESPACE=zivid_camera rosrun nodelet nodelet standalone zivid_camera/nodelet
```

## Using multiple cameras

You can use multiple Zivid cameras simultaneously by starting one node per camera and specifying
unique namespaces:

```
ROS_NAMESPACE=camera1 rosrun zivid_camera zivid_camera_node
```

```
ROS_NAMESPACE=camera2 rosrun zivid_camera zivid_camera_node
```

By default the zivid_camera node will connect to the first available/unused camera. We recommend that
you first start the first node, wait for it to be ready (for example, by waiting for the `capture`
service to be advertised and available), then start the second node. This avoids any race conditions
where both nodes may try to connect to the same camera at the same time.

## How to enable debug logging

The node logs extra information at log level debug, including the settings used when capturing.
Enable debug logging to troubleshoot issues.

```
rosconsole set /<namespace>/zivid_camera ros.zivid_camera debug
```

For example, if ROS_NAMESPACE=zivid_camera,

```
rosconsole set /zivid_camera/zivid_camera ros.zivid_camera debug
```

## Run unit & module tests

This project comes with a set of unit and module tests to verify the provided functionality.
The tests can be run via docker and locally.

### Run tests via docker
```
cd ~/catkin_ws/src/zivid_ros/
OS=ros:melodic-ros-base-bionic ./continuous-integration/ci_test.sh
```

### Run tests locally

First download and install the file camera used for testing.
```
wget -q https://www.zivid.com/software/ZividSampleData.zip
unzip ./ZividSampleData.zip
rm ./ZividSampleData.zip
sudo mkdir -p /usr/share/Zivid/data/
sudo cp ./MiscObjects.zdf /usr/share/Zivid/data/
rm ./MiscObjects.zdf
```

Then run the tests
```
cd ~/catkin_ws && source devel/setup.bash
catkin run_tests && catkin_test_results
```

## Feedback

Please report any issues or feature requests in the issue tracker.

## Acknowledgements

<img src="https://www.zivid.com/software/zivid-ros/rosin_logo.png">

This FTP (Focused Technical Project) has received funding from the European Union’s
Horizon 2020 research and innovation programme under the project ROSIN with the
grant agreement No 732287. For more information, visit [rosin-project.eu](http://rosin-project.eu/).
