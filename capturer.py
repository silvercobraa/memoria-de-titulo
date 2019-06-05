#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import baxter_interface
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np
import time
import sys

from geometry import Position, Orientation
from limb import Limb
from hough import hough

picture = None
DIR = 'capturas_sin_luz/'

class Capturer(object):
    """docstring for Capturer."""

    def __init__(self):
        # self._cam = 'head'
        # self._camera = baxter_interface.CameraController(self._cam + '_camera')
        # self._camera.open()
        # self._camera.resolution = self._camera.MODES[0]
        # self._camera.resolution = (960, 600)
        # self._camera.exposure = 9
        # # self._camera.exposure = 60
        # # self._camera.exposure = 30
        self._cam = cv2.VideoCapture(1)


    def release(self):
        self._cam.release()


    def capture(self, face):
        while True:
            ret, bgr_frame = self._cam.read()
            if not ret or bgr_frame is None:
                print 'None'
                continue
            cv2.imshow('capturas', bgr_frame)
            # gray_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
            # circles = hough(gray_frame)
            circles = hough(bgr_frame)
            if circles is not None and 9 <= circles.shape[1] <= 9:
                np.save('capturas_temp/' + face, bgr_frame)
                np.save('capturas_temp/' + face + '_circles', circles)
                print(bgr_frame)
                print(circles.shape)
                cv2.destroyAllWindows()
                break
            else:
                print 'circulos:', circles

        # def callback(msg):
        #     global picture
        #     picture = cv_bridge.CvBridge().imgmsg_to_cv2(msg)
        #
        # rospy.Subscriber('/cameras/' + self._cam + '_camera/image', Image , callback)
        #
        # while not rospy.is_shutdown():
        #     while np.all(picture) == None:
        #         continue
        #
        #     frame = picture
        #
        #     # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #     circles = hough(frame)
        #     if circles is not None and 9 <= circles.shape[1] <= 9:
        #         np.save(DIR + face, frame)
        #         np.save(DIR + face + '_circles', circles)
        #         print(frame)
        #         print(circles.shape)
        #         break
        #
        #     # cv2.imshow('Imagen', frame)
        #     # rectangles(frame)
        #
        #     #Salir con 'ESC'
        #     k = cv2.waitKey(5) & 0xFF
        #     if k == 27:
        #         break
        # cv2.destroyAllWindows()


def main():
    rospy.init_node('capturer')
    limb = Limb('right')
    capturer = Capturer()
    limb.move(Position.ABOVE, Orientation.UPWARDS_90)
    time.sleep(1)
    limb.move(Position.HEAD_CAMERA, Orientation.UPWARDS_90)
    capturer.capture('F')
    limb.move(Position.HEAD_CAMERA, Orientation.UPWARDS_270)
    capturer.capture('B')
    limb.move(Position.ABOVE, Orientation.UPWARDS_270)
    time.sleep(1)
    pos = (0.35, 0.0, 0.75)
    ori = (0, -3*math.pi/8, 0)
    limb.move(Position.HEAD_CAMERA, Orientation.BACKWARDS_0)
    capturer.capture('D')
    capturer.release()


if __name__ == '__main__':
    main()
