import rospy
import cv2
import numpy as np


class Camera(object):
    def __init__(self, device, size):
        self._cam = cv2.VideoCapture(device)
        x, y = size
        self._cam.set(3, x)
        self._cam.set(4, y)
        self._cam.set(5, 60)

    def read(self):
        return self._cam.read()

    def __del__(self):
        self._cam.release()


class CameraArray(object):
    def __init__(self, devices, size):
        self._cameras = [Camera(device, size) for device in devices]

        # Initialize camera array
        while self.read() is None:
            pass
        rospy.loginfo("crane_vision: Initialized camera array")

    def read(self):
        frames = []
        for cam in self._cameras:
            _, frame = cam.read()
            if frame is None:
                return None
            else:
                frames.append(frame)
        return frames
