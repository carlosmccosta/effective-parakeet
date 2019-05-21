#!/usr/bin/env python

#TODO Find out where to put samples. They should probably not be delivered in the main
# package.

import rospy
import rosnode
import dynamic_reconfigure.client
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

def on_pointcloud(data):
    rospy.loginfo(rospy.get_caller_id() + "PointCloud received!")

def capture_loop():

    rospy.init_node('zivid_sample', anonymous=True)

    rospy.Subscriber("/zivid_camera/pointcloud", PointCloud2, on_pointcloud)

    capture = rospy.ServiceProxy("/zivid_camera/capture", Capture)
    services = [
        s
        for s in dynamic_reconfigure.find_reconfigure_services()
        if s.startswith("/zivid_camera/frame_settings")
    ]
    print(services)

    frame_1_settings_client = dynamic_reconfigure.client.Client("/zivid_camera/frame_settings/frame_0")
    #print(str(frame_1_settings_client.get_configuration()))

    iris = 20
    exposure_time = 6500
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        iris = iris + 1
        exposure_time = exposure_time + 1000

        settings = {"iris": iris, "exposure_time": exposure_time}
        print("Updating camera settings: " + str(settings))
        #frame_1_settings_client.update_configuration(settings)
        print("Calling capture()")
        capture()

        rate.sleep()

if __name__ == '__main__':
    print("STARTING")
    capture_loop()
