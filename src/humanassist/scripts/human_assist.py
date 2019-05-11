import numpy as np
import cv2

from camera_thread import CameraThread
from inference import Inference

from nn.util.vis import view_seg_map

import time
millis = lambda: int(round(time.time() * 1000))

cap = CameraThread(1, size=(432, 240))
cap.start()

model = Inference("/home/ubuntu/model-floornet-trt.pb")

green = (0, 255, 0)
red = (0, 0, 255)
orange = (0, 165, 255)
yellow = (0, 255, 255)

box_near = [(50, 240), (50, 200), (382, 200), (382, 240)]
box_mid = [(50, 200), (50, 170), (382, 170), (382, 200)]
box_far = [(50, 170), (50, 140), (382, 140), (382, 170)]

mask = np.zeros((240, 432, 1), dtype=np.uint8)
cv2.fillPoly(mask, box_near, 3)
cv2.fillPoly(mask, box_mid, 2)
cv2.fillPoly(mask, box_far, 1)

bool_near = (mask == 3)
bool_mid = (mask == 2)
bool_far = (mask == 1)

cumulative_near = bool_near
cumulative_near_len = np.prod(np.array(cumulative_near.shape))

cumulative_mid = bool_near or bool_mid
cumulative_mid_len = np.prod(np.array(cumulative_mid.shape))

cumulative_far = bool_near or bool_mid or bool_far
cumulative_far_len = np.prod(np.array(cumulative_far.shape))

while True:
    frame = cap.read()
    seg = model.infer(frame)

    print(seg.shape)

    if len(seg.shape) > 2 and seg.shape[2] > 1:
        seg = cv2.cvtColor(seg, cv2.COLOR_BGR2GRAY)

    print(seg.shape)

    if len(seg.shape) < 3:
        seg = np.expand_dims(seg, axis = 2)

    print(seg.shape)

    overlay = frame.copy()
    overlay[np.where((seg > 0).all(axis = 2))] = green
    overlay[np.where(((seg == 0) and bool_near).all(axis = 2))] = red
    overlay[np.where(((seg == 0) and bool_mid).all(axis = 2))] = orange
    overlay[np.where(((seg == 0) and bool_far).all(axis = 2))] = yellow

    alpha = 0.4

    output = frame.copy()
    cv2.addWeighted(overlay, alpha, output, 1 - alpha, 0, output)

    near = float(((seg == 0) and cumulative_near).sum()) / cumulative_near_len
    mid = float(((seg == 0) and cumulative_mid).sum()) / cumulative_mid_len
    far = float(((seg == 0) and cumulative_far).sum()) / cumulative_far_len

    print("{} | {} | {}".format(round(near, 2), round(mid, 2), round(far, 2)))

    #vis = view_seg_map(frame, seg, color=(0, 255, 0), alpha=0.3)

    cv2.imshow("Image", output)
    cv2.waitKey(5)
