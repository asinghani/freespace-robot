#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray, Empty

import numpy as np
import cv2

from camera_thread import CameraThread
from inference import Inference

from nn.util.vis import view_seg_map

import time
millis = lambda: int(round(time.time() * 1000))

rospy.init_node("freespace_safety")
obsPub = rospy.Publisher("/obstacles", Float32MultiArray, queue_size=10)
soundPub = rospy.Publisher("/sound", Empty, queue_size=10)

cap = CameraThread(1, size=(432, 240))
cap.start()

model = Inference("/home/ubuntu/model-floornet-trt.pb")

green = (0, 255, 0)
red = (0, 0, 255)
orange = (0, 165, 255)
yellow = (0, 255, 255)

box_near = [(100, 240), (186, 170), (255, 170), (342, 240)]
box_mid = [(186, 170), (186, 170), (255, 170), (255, 170)]
box_far = [(186, 170), (220, 142), (255, 170)]

line1 = [(100, 240), (186, 170)]
line2 = [(342, 240), (255, 170)]

mask = np.zeros((240, 432, 1), dtype=np.uint8)
cv2.fillPoly(mask, np.array([box_near], dtype=np.int32), 3)
cv2.fillPoly(mask, np.array([box_mid], dtype=np.int32), 2)
cv2.fillPoly(mask, np.array([box_far], dtype=np.int32), 1)

bool_near = (mask == 3)
bool_mid = (mask == 2)
bool_far = (mask == 1)

cumulative_near = bool_near
cumulative_near_len = cumulative_near.sum()

cumulative_mid = np.logical_or(bool_near, bool_mid)
cumulative_mid_len = cumulative_mid.sum()

cumulative_far = np.logical_or(cumulative_mid, bool_far)
cumulative_far_len = cumulative_far.sum()

while True:
    frame = cap.read()
    seg = model.infer(frame)
    seg = np.expand_dims(seg, axis = 2)

    near = float(np.logical_and((seg == 0), cumulative_near).sum()) / cumulative_near_len
    mid = float(np.logical_and((seg == 0), cumulative_mid).sum()) / cumulative_mid_len
    far = float(np.logical_and((seg == 0), cumulative_far).sum()) / cumulative_far_len

    obsPub.publish(Float32MultiArray(data=[near, mid, far]))

    alpha = 0.4

    overlay = frame.copy()
    overlay[np.where((seg > 0).all(axis = 2))] = green
    overlay[np.where(np.logical_and((seg == 0), bool_near).all(axis = 2))] = red
    overlay[np.where(np.logical_and((seg == 0), bool_mid).all(axis = 2))] = orange
    overlay[np.where(np.logical_and((seg == 0), bool_far).all(axis = 2))] = yellow

    output = frame.copy()
    cv2.addWeighted(overlay, alpha, output, 1 - alpha, 0, output)

    cv2.line(output, line1[0], line1[1], (200, 0, 0), 3)
    cv2.line(output, line2[0], line2[1], (200, 0, 0), 3)

    if mid > 0.3:
        # play sound
        soundPub.publish(Empty())

    cv2.imwrite("/home/ubuntu/photos/i{}.png".format(millis()), output)
    #cv2.waitKey(5)
