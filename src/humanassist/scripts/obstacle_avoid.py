#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

SCALE_SPEED = 0.5
SCALE_STEER = 0.65

near = 0.0
mid = 0.0
far = 0.0

def obsCallback(msg):
    global near, mid, far
    (near, mid, far) = msg.data

speed = 0.0
steer = 0.0
def joyCallback(msg):
    global speed, steer
    speed = msg.axes[1] * SCALE_SPEED
    steer = msg.axes[0] * SCALE_STEER

rospy.init_node("obstacle_avoid")

twistPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
joySub = rospy.Subscriber("/joy", Joy, joyCallback)
obsSub = rospy.Subscriber("/obstacles", Float32MultiArray, obsCallback)

while not rospy.is_shutdown():
    newMsg = Twist()
    newMsg.angular.z = steer

    # stop if unsafe
    if near > 0.1:
        newMsg.linear.x = min(speed, 0.0)
    elif mid > 0.3:
        if speed > 0.0:
            newMsg.linear.x = speed / 2.0
    else:
        newMsg.linear.x = speed

    twistPub.publish(newMsg)

    time.sleep(0.05)
