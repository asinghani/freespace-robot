import numpy as np
import cv2

from camera_thread import CameraThread
from inference import Inference

from nn.util.vis import view_seg_map

import time
millis = lambda: int(round(time.time() * 1000))

cap = CameraThread(1, size=(480, 270))
cap.start()

model = Inference("/home/ubuntu/model-floornet-trt.pb")

while True:
    frame = cap.read()

    seg = model.infer(frame)

    vis = view_seg_map(frame, seg, color=(0, 255, 0), alpha=0.3)

    cv2.imshow("Image", vis)
    cv2.waitKey(5)
