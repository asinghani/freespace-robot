#!/usr/bin/env python

import rospy
import yaml
from sensor_msgs.msg import CameraInfo

file = rospy.get_param("~calibration_yaml")

data = yaml.load(file)

msg = CameraInfo()
msg.width = data["image_width"]
msg.height = data["image_height"]
msg.K = data["camera_matrix"]["data"]
msg.D = data["distortion_coefficients"]["data"]
msg.R = data["rectification_matrix"]["data"]
msg.P = data["projection_matrix"]["data"]
msg.distortion_model = data["distortion_model"]

rospy.init_node("camera_info_publisher", anonymous=True)
pub = rospy.Publisher("camera_info", CameraInfo, queue_size=10)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(mdg)
    rate.sleep()
