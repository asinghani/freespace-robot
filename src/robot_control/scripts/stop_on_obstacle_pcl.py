#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist

pclMsg = None
def pclCallback(msg):
    global pclMsg
    pclMsg = msg

twistMsg = None
def twistCallback(msg):
    global twistMsg
    twistMsg = msg

rospy.init_node("obstacle_stop")

twistPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
twistSub = rospy.Subscriber("/command", Twist, twistCallback)

pclSub = rospy.Subscriber("/pcl", PointCloud, pclCallback)

width = float(rospy.get_param("width", 0.6)) / 2.0 # meters, (half to left half to right)
thresh = float(rospy.get_param("threshold_distance", 0.55))

def safe(): # check if safe to proceed forward (given lidar data)
    if pclMsg is None:
        return True
    points = [abs(point.y) for point in pclMsg.points if abs(point.x) < width]
    points = [x for x in points if x > 0.03 and x < 15]
    if len(points) == 0:
        return True
    avg = sum(points) / float(len(points))

    return avg > thresh

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
