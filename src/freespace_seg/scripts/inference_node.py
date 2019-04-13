#!/usr/bin/env python

import tensorflow as tf
print(tf.__version__)

import time
import numpy as np
import cv2

import rospy

from camera_model import *
from inference import Inference
from image_publisher import ImagePublisher

from imutils.video import WebcamVideoStream
from imutils.video import FPS

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg

rospy.init_node("inference_node")

# CAMERA OPEN -> MODEL LOAD -> CAMERA INFO LOAD -> LOOP (UNDISTORT -> INFER -> PCL -> PUBLISH PCL)

# Set up camera
img_shape = np.array(rospy.get_param("/img_shape"), dtype=np.uint8) # width, height, (channels)
cap = WebcamVideoStream(src=rospy.get_param("/camera_id", 1)).start()

camera_type_fisheye = rospy.get_param("/camera_type", "pinhole").lower() == "fisheye"
distortion_coefficients = np.array(rospy.get_param("/distortion_coefficients", [0.0, 0.0, 0.0, 0.0]), dtype=np.float32)
camera_matrix = np.array(rospy.get_param("/camera_matrix"), dtype=np.float32).reshape((3, 3))
camera_height = float(rospy.get_param("/camera_height"))
camera_config = CameraConfig.FromCameraMatrix(camera_matrix, camera_height)

model_path = rospy.get_param("/model_path", None)
model_input_shape = np.array(rospy.get_param("/input_shape"), dtype=np.uint)

pointcloud_topic = rospy.get_param("/pointcloud_topic", "/pcl")
pointcloud_frame = rospy.get_param("/pointcloud_frame", "pcl")
visualization_topic = rospy.get_param("/visualization_topic", None)
include_visualization = (visualization_topic is not None) and (visualization_topic.lower() != "none")

inference = Inference(model_input_shape, model_path)
image_pub = ImagePublisher(visualization_topic, True)
pcl_pub = rospy.Publisher(pointcloud_topic, PointCloud, queue_size=2)

while not rospy.is_shutdown():
    frame = cap.read()

    if camera_type_fisheye:
        dims = (640, 480)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(camera_matrix, distortion_coefficients, np.eye(3), camera_matrix, dims, cv2.CV_16SC2)
        frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    else:
        frame = cv2.undistort(frame, camera_matrix, distortion_coefficients)

    original_shape = frame.shape[0:2]
    frame = cv2.resize(frame, tuple(model_input_shape))
    prediction, visualization = inference.predict(frame, True)

    image_pub.publish(visualization, True)

    prediction = cv2.resize(prediction, (640, 480)).argmax(axis=2)
    border = get_segmentation_border(prediction, every_n=5)
    print(border.shape)
    #border = np.array([0, 480], dtype=np.float32) - np.array(border, dtype=np.float32)
    #border = np.abs(border)
    #border = border.reshape((border.shape[0], 1, 2))
    #border = cv2.undistortPoints(border, camera_matrix, distortion_coefficients)
    #border = border.reshape((border.shape[0], 2))
    #print(border.shape)
    #print(border)
    rays = multi_raycast(border, camera_config)

    pcl = PointCloud()
    pcl.header = std_msgs.msg.Header()
    pcl.header.stamp = rospy.Time.now()
    pcl.header.frame_id = pointcloud_frame

    for x, y, z in rays:
        pcl.points.append(Point32(x, z, y))

    pcl_pub.publish(pcl)

