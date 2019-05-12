import cv2
import numpy as np
import time

current_milli_time = lambda: int(round(time.time() * 1000))

cap = cv2.VideoCapture(1)
cap.set(3, 1024)
cap.set(4, 576)

_, frame = cap.read()
out = cv2.VideoWriter("vid.avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (frame.shape[1], frame.shape[0]))

x = 0
r = current_milli_time()
while x < 200:
    _, frame = cap.read()
    out.write(frame)
    cv2.waitKey(1)
    print(x, current_milli_time() - r)
    x = x + 1

cap.release()
out.release()
