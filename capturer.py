#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

class Capturer(object):
    limb = Limb('left')
    """docstring for Capturer."""

    def __init__(self):
        pass


def main():
    rospy.init_node('capturer')
    limb = Limb('left')
    # limb.move(Position.HEAD_CAMERA, Orientation.BACKWARDS_0)
    # sys.exit()
    # limb.move(Position.ABOVE, Orientation.UPWARDS_90)
    # time.sleep(1)
    limb.move(Position.HEAD_CAMERA, Orientation.UPWARDS_90)
    # time.sleep(1)
    # limb.move(Position.HEAD_CAMERA, Orientation.UPWARDS_270)
    # time.sleep(1)
    # limb.move(Position.ABOVE, Orientation.UPWARDS_270)

    cam = 'head'
    lhc = baxter_interface.CameraController(cam + '_camera')
    lhc.open()
    lhc.resolution = lhc.MODES[0]
    lhc.resolution = (960, 600)
    lhc.exposure = 60
    # lhc.exposure = 30

    def callback(msg):
        # Transforma el mensaje a imagen
        global picture
        picture = cv_bridge.CvBridge().imgmsg_to_cv2(msg) #, "bgr8") #bgr8

    rospy.Subscriber('/cameras/' + cam + '_camera/image', Image , callback)

    while not rospy.is_shutdown():
        # Capturar un frame
        while np.all(picture) == None:
            print "hola"
            continue

        frame = picture

        #Mostrar la imagen
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = hough(gray_frame)
        # if circles is not None and 9 <= circles.shape[1] <= 9:
        #     # np.save('capturas/U', frame)
        #     # np.save('capturas/U_circles', circles)
        #     print(frame)
        #     print(circles.shape)
            # break
        # rectangles(frame)
        # cv2.imshow('Imagen', frame)

        #Salir con 'ESC'
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
