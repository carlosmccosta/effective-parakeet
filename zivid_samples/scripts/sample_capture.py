#!/usr/bin/env python

# TODO Find out where to put samples. They should probably not be delivered in the main
# package.

import rospy
import rosnode
import dynamic_reconfigure.client
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2


def on_point_cloud(data):
    rospy.loginfo("PointCloud received")


def capture_loop():

    rospy.init_node("zivid_sample", anonymous=True)

    rospy.Subscriber("/zivid_camera/point_cloud", PointCloud2, on_point_cloud)

    capture = rospy.ServiceProxy("/zivid_camera/capture", Capture)

    frame_0_config_client = dynamic_reconfigure.client.Client(
        "/zivid_camera/capture_frame/frame_0"
    )

    rospy.loginfo("Enable and configure the first frame")
    settings = {"enabled": True, "iris": 22, "exposure_time": 0.02}
    frame_0_config_client.update_configuration(settings)

    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        rospy.loginfo("Calling capture")
        capture()
        rate.sleep()


if __name__ == "__main__":
    capture_loop()
