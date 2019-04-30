import cv2
import numpy as np 


class Camera(object):
    def __init__(self, device, size):
        self._cam = cv2.VideoCapture(device)
        x, y = size
        self._cam.set(3, x)
        self._cam.set(4, y)

    def read(self):
        return self._cam.read()