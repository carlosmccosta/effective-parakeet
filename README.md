# Zivid ROS driver

[![Build Status](https://travis-ci.org/nedrebo/effective-parakeet.svg?branch=master)](https://travis-ci.org/nedrebo/effective-parakeet)

THIS DRIVER IS WORK IN PROGRESS. IT WILL CHANGE BEFORE FINAL RELEASE.

This is the official ROS driver for Zivid 3D cameras. Read more about Zivid at [our website](https://www.zivid.com/).

<p align="center">
    <img src="https://www.zivid.com/software/zivid-ros/ros_zivid_camera.png" width="600" height="273">
</p>

## Installation
Follow this step-by-step guide to install the Zivid ROS driver on your system. This driver supports
Ubuntu 16.04 with ROS Kinetic and Ubuntu 18.04 with ROS Melodic.

### Prerequisites

#### ROS
Follow the [ROS installation wiki](http://wiki.ros.org/ROS/Installation) to install ROS Kinetic (for Ubuntu 16.04)
or ROS Melodic (for Ubuntu 18.04).

Also install catkin and git.

```
sudo apt-get update
sudo apt-get install -y python-catkin-tools git
```

#### OpenCL
An OpenCL 1.2 compatible GPU and OpenCL driver is required by the Zivid Core library.
Follow the [guide in our knowledge base](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/426519/Install+OpenCL+drivers+on+Ubuntu) to install OpenCL for your system.

#### Zivid Core Library
Download and install the "Toshiba Teli driver" and "Zivid Core" debian packages from
[our webpage](https://www.zivid.com/downloads). Zivid Core version 1.3 or newer is required.
Optionally install the "Zivid Studio" and "Zivid Tools" packages as well. They are not required by
the ROS driver but can be useful for testing that your system has been setup correctly and that
the camera is detected.

#### C++ compiler

A C++17 compiler is required.

Ubuntu 16.04:
```
sudo apt-get install -y software-properties-common
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install -y g++-8
```

Ubuntu 18.04:
```
sudo apt-get install -y g++
```

#### Create catkin workspace

If you have not created a catkin workspace, this needs to be done first.

Ubuntu 16.04:
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws && catkin build
```

Ubuntu 18.04:
```
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws && catkin build
```

### Downloading and building Zivid ROS driver

Clone the Zivid ROS project into the ~/catkin_ws/src directory.
```
cd ~/catkin_ws/src
git clone https://github.com/nedrebo/effective-parakeet.git
```

Install dependencies.
```
cd ~/catkin_ws
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Finally, build the `zivid_camera` and `zivid_samples` packages.

Ubuntu 16.04:
```
catkin build -DCMAKE_CXX_COMPILER=/usr/bin/g++-8
```

Ubuntu 18.04:
```
catkin build
```

## Getting started

Connect the Zivid camera to your USB3 port on your PC. You can use the ZividListCameras tool
available in the zivid-tools package to confirm that your system has been configured correctly, and
that the camera is discovered by your PC. You can also open Zivid Studio and connect to the camera.
Close Zivid Studio before continuing with the rest of this guide. If the camera is not found, visit
our [troubleshooting wiki](https://help.zivid.com).

Run the sample_capture.launch script via [roslaunch](https://wiki.ros.org/roslaunch) to test that
everything is working:

```
cd ~/catkin_ws && source devel/setup.bash
roslaunch zivid_samples sample_capture.launch
```

The `zivid_samples_capture` node will first configure the capture settings of the camera and then
capture repeatedly. The launch script also starts [rviz](https://wiki.ros.org/rviz) to visualize the
point cloud and the 2D color and depth images, and [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure)
to configure the capture settings.

<p align="center">
    <img src="https://www.zivid.com/software/zivid-ros/ros_rviz_miscobjects.png" width="800" height="472">
</p>

If everything is working, the camera will now start to capture frames repeatedly, and the output should
be visible in [rviz](https://wiki.ros.org/rviz). Try to adjust the exposure time or the iris in
[rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure) and observe that the visualization in
[rviz](https://wiki.ros.org/rviz) changes. Note: sometimes it is necessary to click "Refresh" in
[rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure) to load the configuration tree.

A more detailed description of the `zivid_camera` node follows below.

For sample code in C++ and Python, see the Samples section.

## Launching the driver

It is required by the driver to specify a namespace when starting the driver. All the
services, topics and configurations will be pushed into this namespace.

### As a node

```
ROS_NAMESPACE=zivid_camera rosrun zivid_camera zivid_camera_node
```

### As a nodelet

```
ROS_NAMESPACE=zivid_camera rosrun nodelet nodelet standalone zivid_camera/nodelet
```

## Launch Parameters

The following parameters can be specified when starting the driver.

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

## Published Topics

`color/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
> Camera calibration and metadata.

`color/image_color` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
> RGB image. The image is encoded as "rgb8".

`depth/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
> Camera calibration and metadata.

`depth/image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
> Depth image. Each pixel contains the z-value (along the camera Z axis) in meters.
> The image is encoded as 32-bit float. Pixels where z-value is missing are NaN.

`depth/points` ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))
> Point cloud data. Each time a capture is invoked the resulting point cloud is published
> on this topic. The included point fields are x, y, z (in meters), c (contrast value),
> and r, g, b (colors). The output is in the camera's optical frame, where x is right, y is
> down and z is forward.

## Configuration

The capture settings can be configured using [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure).
Use [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure) to view or change the settings using a GUI.

```
rosrun rqt_reconfigure rqt_reconfigure
```

The capture settings available in the `zivid_camera` node matches the settings in the Zivid API.
To become more familiar with the available settings and what they do, see the
[API reference](http://www.zivid.com/software/api-documentation) for the `Settings` class or use
Zivid Studio.

Note that the available settings will depend on which version of Zivid Core you are using.

The available capture settings are organized into a hierarchy:

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

The `zivid_camera` node supports both single-capture and HDR-capture. HDR-capture works by taking
several individual captures (called frames) with different settings (for example different exposure time)
and combining the captures into one high-quality point cloud.

For more information about HDR capture, visit our [knowledge base](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/428143/HDR+Imaging+for+Challenging+Objects).

Note that the min, max and default value of the dynamic_reconfigure parameters can change dependent on
what Zivid camera model you are using. Therefore you should not use the `__getMin()__`, `__getMax()__` and
`__getDefault()__` methods of the auto-generated C++ config classes (`zivid_camera::CaptureGeneralConfig`
and `zivid_camera::CaptureFrameConfig`). Instead you should query the server for the default values.
See the sample code for how to do this.

### Frame settings

`frame_settings/frame_<n>/` contains settings for an individual frame. By default `<n>` can be 0 to 9
for a total of 10 configured frames. The total number of frames can be configured using the launch
parameter `num_capture_frames` (see section Launch Parameters below).

`frame_settings/frame_<n>/enabled` controls if frame `<n>` will be included when the `capture/` service is
invoked. If only one frame is enabled the `capture/` service performs a single-capture. If more than
one frame is enabled the `capture/` service will perform an HDR-capture. By default enabled is false.
In order to capture a point cloud at least one frame needs to be enabled.

| Name                                          | Type   |  Zivid API Setting             |   Note   |
|-----------------------------------------------|--------|----------------------------------------------|----------|
| `frame_settings/frame_<n>/bidirectional`      | bool   | [Zivid::Settings::Bidirectional](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Bidirectional.html)
| `frame_settings/frame_<n>/brightness`         | double | [Zivid::Settings::Brightness](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Brightness.html)
| `frame_settings/frame_<n>/enabled`            | bool |  |
| `frame_settings/frame_<n>/exposure_time`      | double | [Zivid::Settings::ExposureTime](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1ExposureTime.html) | Specified in seconds
| `frame_settings/frame_<n>/gain`               | double | [Zivid::Settings::Gain](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Gain.html)
| `frame_settings/frame_<n>/iris`               | int    | [Zivid::Settings::Iris](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Iris.html)


### General capture settings

`capture_general` contains settings that apply to all frames in a capture.

| Name                                          | Type   |  Zivid API Setting             |
|-----------------------------------------------|--------|----------------------------------------------|
| `capture_general/blue_balance`                | double | [Zivid::Settings::BlueBalance](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1BlueBalance.html)
| `capture_general/filters_contrast_enabled`    | bool   | [Zivid::Settings::Filters::Contrast::Enabled](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Contrast_1_1Enabled.html)
| `capture_general/filters_contrast_threshold`  | double | [Zivid::Settings::Filters::Contrast::Threshold](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Contrast_1_1Threshold.html)
| `capture_general/filters_gaussian_enabled`    | bool   | [Zivid::Settings::Filters::Gaussian::Enabled](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Gaussian_1_1Enabled.html)
| `capture_general/filters_gaussian_sigma`      | double | [Zivid::Settings::Filters::Gaussian::Sigma](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Gaussian_1_1Sigma.html)
| `capture_general/filters_outlier_enabled`     | bool   | [Zivid::Settings::Filters::Outlier::Enabled](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Outlier_1_1Enabled.html)
| `capture_general/filters_outlier_threshold`   | double | [Zivid::Settings::Filters::Outlier::Threshold](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Outlier_1_1Threshold.html)
| `capture_general/filters_reflection_enabled`  | bool   | [Zivid::Settings::Filters::Reflection::Enabled](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Reflection_1_1Enabled.html)
| `capture_general/filters_saturated_enabled`   | bool   | [Zivid::Settings::Filters::Saturated::Enabled](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Saturated_1_1Enabled.html)
| `capture_general/red_balance`                 | double | [Zivid::Settings::RedBalance](https://www.zivid.com/software/releases/1.3.0+bb9ee328-10/doc/cpp/classZivid_1_1Settings_1_1RedBalance.html)

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

## Frequently Asked Questions

### How to use multiple cameras

You can use multiple Zivid cameras simultaneously by starting one node per camera and specifying
unique namespaces per node:

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

### How to run the unit and module tests

This project comes with a set of unit and module tests to verify the provided functionality. To run
the tests locally, first download and install the file camera used for testing:
```
wget -q https://www.zivid.com/software/ZividSampleData.zip
unzip ./ZividSampleData.zip
rm ./ZividSampleData.zip
sudo mkdir -p /usr/share/Zivid/data/
sudo cp ./MiscObjects.zdf /usr/share/Zivid/data/
rm ./MiscObjects.zdf
```

Then run the tests:
```
cd ~/catkin_ws && source devel/setup.bash
catkin run_tests && catkin_test_results
```

The tests can also be run via docker. See the [Travis configuration file](./.travis.yml) for details.

### How to enable debug logging

The node logs extra information at log level debug, including the settings used when capturing.
Enable debug logging to troubleshoot issues.

```
rosconsole set /<namespace>/zivid_camera ros.zivid_camera debug
```

For example, if ROS_NAMESPACE=zivid_camera,

```
rosconsole set /zivid_camera/zivid_camera ros.zivid_camera debug
```

## Feedback

Please report any issues or feature requests in the issue tracker.

## Acknowledgements

<img src="https://www.zivid.com/software/zivid-ros/rosin_logo.png">

This FTP (Focused Technical Project) has received funding from the European Union’s
Horizon 2020 research and innovation programme under the project ROSIN with the
grant agreement No 732287. For more information, visit [rosin-project.eu](http://rosin-project.eu/).
