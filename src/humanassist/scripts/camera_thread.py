import cv2
from threading import Thread
class CameraThread:
    def __init__(self, camera, size=None, preprocess_func=lambda x: x):
        self.cap = cv2.VideoCapture(camera)

        if size is not None:
            self.cap.set(3, size[0])
            self.cap.set(4, size[1])

        _, self.frame = self.cap.read()

    def start(self):
        th = Thread(target=self.loop, args=())
        th.dameon = True
        th.start()
        return self

    def loop(self):
        while True:
            success, frame = self.cap.read()
            if success:
                self.frame = frame

    def read(self):
        return self.frame

