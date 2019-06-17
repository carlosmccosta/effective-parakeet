#!/usr/bin/env python

import rospy
import rosnode
import dynamic_reconfigure.client
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2


class Sample:
    def __init__(self):
        rospy.init_node("zivid_sample", anonymous=True)

        rospy.Subscriber("/zivid_camera/depth/points", PointCloud2, self.on_point_cloud)

        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)

        rospy.loginfo("Enable the reflection filter")
        general_config_client = dynamic_reconfigure.client.Client(
            "/zivid_camera/capture_general/"
        )
        general_config = {"filters_reflection_enabled": True}
        general_config_client.update_configuration(general_config)

        rospy.loginfo("Enable and configure the first frame")
        frame0_config_client = dynamic_reconfigure.client.Client(
            "/zivid_camera/capture_frame/frame_0"
        )
        frame0_config = {"enabled": True, "iris": 22, "exposure_time": 0.01}
        frame0_config_client.update_configuration(frame0_config)

    def capture(self):
        rospy.loginfo("Calling capture service")
        self.capture_service()

    def on_point_cloud(self, data):
        rospy.loginfo("PointCloud received")
        self.capture()


if __name__ == "__main__":
    s = Sample()
    s.capture()
    rospy.spin()
