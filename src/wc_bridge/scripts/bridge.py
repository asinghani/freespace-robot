#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import time

TIME_MILLIS = lambda: int(round(time.time() * 1000))

def convert(linvel, angvel):
    # Converts linvel, angvel (m/s, rad/s) into speed and steer values for drivetrain
    return (linvel, -angvel)

rospy.init_node("wc_bridge")

speedPub = rospy.Publisher("/wc_controller/speed", Float32, queue_size=10)
steerPub = rospy.Publisher("/wc_controller/steer", Float32, queue_size=10)

last = 0

def callback(twist):
    global last
    speed, steer = convert(twist.linear.x, twist.angular.z)
    speedPub.publish(speed)
    steerPub.publish(steer)
    last = TIME_MILLIS()

twistSub = rospy.Subscriber("/cmd_vel", Twist, callback)

while not rospy.is_shutdown():
    if TIME_MILLIS() - last > 500: # 0.5 second timeout
        speedPub.publish(0.0)
        steerPub.publish(0.0)

    time.sleep(0.05)
