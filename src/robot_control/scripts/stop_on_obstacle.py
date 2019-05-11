#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

lidarMsg = None
def lidarCallback(msg):
    global lidarMsg
    lidarMsg = msg

twistMsg = None
def twistCallback(msg):
    global twistMsg
    twistMsg = msg

rospy.init_node("obstacle_stop")

twistPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
twistSub = rospy.Subscriber("/command", Twist, twistCallback)

lidarSub = rospy.Subscriber("/scan", LaserScan, lidarCallback)

center = int(rospy.get_param("center", 0)) # degrees
width = int(rospy.get_param("width", 20)) # degrees
thresh = float(rospy.get_param("threshold_distance", 1.5))

def safe(): # check if safe to proceed forward (given lidar data)
    if lidarMsg is None:
        return True
    points = lidarMsg.ranges[0:10] + lidarMsg.ranges[-10:]
    points = [x for x in points if x > 0.03 and x < 15]
    avg = sum(points) / float(len(points))

    return avg > 0.55

while not rospy.is_shutdown():
    if twistMsg is None:
        twistPub.publish(Twist())
    else:
        newMsg = Twist()
        newMsg.angular.z = twistMsg.angular.z
        if not safe():
            newMsg.linear.x = min(twistMsg.linear.x, 0.0)
        else:
            newMsg.linear.x = twistMsg.linear.x
        twistPub.publish(newMsg)


    time.sleep(0.05)
