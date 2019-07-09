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


class Capturer(object):
    # BRIGHTNESS = 0.1
    # BRIGHTNESS = 0.2
    # BRIGHTNESS = 0.3
    BRIGHTNESS = 0.5
    """docstring for Capturer."""

    def __init__(self, dir, camera_id):
        self._dir = dir
        self._cam_id = camera_id


    def release(self):
        self._cam.release()


    def capture(self, face):
        while True:
            self._cam = cv2.VideoCapture(self._cam_id)
            self._cam.set(cv2.cv.CV_CAP_PROP_BRIGHTNESS, Capturer.BRIGHTNESS)
            ret, bgr_frame = self._cam.read()
            if not ret or bgr_frame is None:
                print 'None'
                continue

            if cv2.waitKey(33) >= 0:
                break

            circles = hough(bgr_frame)
            if circles is not None and 9 <= circles.shape[1] <= 9:
                np.save(self._dir + face, bgr_frame)
                np.save(self._dir + face + '_circles', circles)
                print(bgr_frame)
                print(circles.shape)
                cv2.destroyAllWindows()
                break
            else:
                print 'circulos:', circles

            self.release()


def main():
    # rospy.init_node('capturer')
    # limb = Limb('left')
    capturer = Capturer('capturas/', 1)
    # limb.move(Position.ABOVE, Orientation.UPWARDS_90)
    # time.sleep(1)
    # limb.move(Position.HEAD_CAMERA, Orientation.UPWARDS_90)
    # capturer.capture('F')
    # limb.move(Position.HEAD_CAMERA, Orientation.UPWARDS_270)
    # capturer.capture('B')
    # limb.move(Position.ABOVE, Orientation.UPWARDS_270)
    # time.sleep(1)
    # pos = (0.35, 0.0, 0.75)
    # ori = (0, -3*math.pi/8, 0)
    # limb.move((0.40, 0, 0.65), (0, -math.pi/4, 0))
    # capturer.capture('D')

    capturer.capture('F')
    time.sleep(3)
    capturer.capture('B')
    time.sleep(3)
    capturer.capture('D')
    time.sleep(3)
    capturer.capture('R')
    time.sleep(3)
    capturer.capture('L')
    time.sleep(3)
    capturer.capture('U')
    time.sleep(3)

    capturer.release()


if __name__ == '__main__':
    main()
