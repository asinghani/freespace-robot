#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import CameraInfo, PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg

import yaml
import time
from math import sin, cos, tan, isnan
import image_geometry as geo

lidarProj = lg.LaserProjection()

# Load calibration
f = "/home/ubuntu/FreespaceProject/calib/c920.yaml"
data = None
with open(f, "r") as stream:
    data = yaml.load(stream)

calib = CameraInfo()
calib.width = data["image_width"]
calib.height = data["image_height"]
calib.K = data["camera_matrix"]["data"]
calib.D = data["distortion_coefficients"]["data"]
calib.R = data["rectification_matrix"]["data"]
calib.P = data["projection_matrix"]["data"]
calib.distortion_model = data["distortion_model"]

lidarMsg = None
def lidarCallback(msg):
    global lidarMsg
    #lidarMsg = lidarProj.projectLaser(msg)
    lidarMsg = msg

rospy.init_node("draw_lidar_points", anonymous=True)
lidarSub = rospy.Subscriber("/scan", LaserScan, lidarCallback)

start = -0.5 # pi/6 = -30 deg
end = 0.5 # pi/6 = 30 deg
Y = -0.2

while lidarMsg is None:
    time.sleep(0.1)

#startIndex = int((start - lidarMsg.angle_min) / lidarMsg.angle_increment)
#endIndex = int((end - lidarMsg.angle_min) / lidarMsg.angle_increment)
inc = lidarMsg.angle_increment

cap = cv2.VideoCapture(1)

model = geo.PinholeCameraModel()
model.fromCameraInfo(calib)
print(model, model.cx(), model.cy())

#320, 430


# -0.152, 0.546

print(model.projectPixelTo3dRay((320, 480-430)))
print(model.projectPixelTo3dRay((480-430, 320)))


while True:
    img = cap.read()[1]
    img2 = img.copy()

    model.rectifyImage(img2, img)

    #data = pc2.read_points(lidarMsg)
    data = lidarMsg.ranges

    if len(data) < int((lidarMsg.angle_max - lidarMsg.angle_min) / inc):
        continue

    data = [(data[i] * cos(lidarMsg.angle_min + i * inc), data[i] * sin(lidarMsg.angle_min + i * inc)) for i in range(0, int((lidarMsg.angle_max - lidarMsg.angle_min) / inc))]

    points = [model.project3dToPixel((point[1], -0.147, -point[0] - 0.05)) for point in data if -point[0] > 0]

    #points = [model.project3dToPixel((0.072, -0.147, 0.4572))]

    """data = pc2.read_points(lidarMsg)
    sum1 = 0.0
    sum2 = 0.0
    sum3 = 0.0
    num = 0.0
    for point in data:
        #print(point)
        if not isnan(point[0]):
            sum1 += point[0]
            sum2 += point[1]
            sum3 += point[2]
            num += 1


    # we can calculate the average z value for example
    print(str(sum1/num),str(sum2/num),str(sum3/num))

    print(points[::40])
    points = points[::1]
"""
    points = [(int(x), 480 - int(y)) for x, y in points if (not isnan(x) and not isnan(y))]


    """data = pc2.read_points(lidarMsg)
    cv2.circle(img, (320, 240), 6, (255, 255, 0), -1)
    for pt in data:
        #print(pt)
        cv2.circle(img, (int(pt[0] * 100 + 320), int(pt[1] * 100 + 240)), 1, (0, 255, 0), -1)
        """

    for pt in points:
        cv2.circle(img, pt, 2, (0, 255, 0), -1)
        #cv2.line(img, (0, pt[1]), (640, pt[1]), (255, 0, 0), 3)
        #cv2.line(img, (pt[0], 0), (pt[0], 480), (255, 0, 0), 3)

    print(" -- ")

    cv2.imshow("img", img)
    cv2.waitKey(20)
