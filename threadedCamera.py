import cv2 as cv
import numpy as np
from threading import Thread

class threadedCamera:
    def __init__(self, cam='http://169.229.96.70:8080/?action=stream'):
        self.cam = cv.VideoCapture(cam)
        self._ret, self._frame = self.cam.read()
        self.thread = Thread(target=self.start)
        self.thread.start()
    def start(self):
        while self.cam.isOpened():
            self._ret, self._frame = self.cam.read()
    def stop(self):
        self.cam.release()
    @property
    def frame(self): return self._frame
    def read(self):
        return self._ret, self._frame
