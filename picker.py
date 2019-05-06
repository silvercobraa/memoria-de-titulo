import cv2
import rospy
import cv_bridge
import baxter_interface
from sensor_msgs.msg import Image
import time
import numpy as np

from limb import Limb
from geometry import Position, Orientation
from hough import hough, rectangles

picture = None

def main():
    # rosservice call /cameras/open '{name: left_hand_camera, settings: {width: 960, height: 600}}'
    rospy.init_node('picker')
    side = 'left'
    # limb = Limb('left')
    # limb.move([0.5, Position.Y, 0], Orientation.DOWNWARDS)
    lhc = baxter_interface.CameraController(side + '_hand_camera')
    lhc.open()
    lhc.resolution = lhc.MODES[0]
    lhc.exposure = 60
    # lhc.exposure = 30

    def callback(msg):
        # Transforma el mensaje a imagen
        global picture
        picture = cv_bridge.CvBridge().imgmsg_to_cv2(msg) #, "bgr8") #bgr8

    rospy.Subscriber('/cameras/' + side + '_hand_camera/image', Image , callback)

    while not rospy.is_shutdown():
        # Capturar un frame
        while np.all(picture) == None:
            # print "hola"
            continue

        frame = picture

        #Mostrar la imagen
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = hough(gray_frame)
        if circles is not None and 9 <= circles.shape[1] <= 9:
            np.save('capturas/U', frame)
            np.save('capturas/U_circles', circles)
            print(frame)
            print(circles.shape)
            break
        # rectangles(frame)
        # cv2.imshow('Imagen', frame)

        #Salir con 'ESC'
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()
    # limb.move(Position.CUBE_POSITION, Orientation.DOWNWARDS)
    # limb.close()
    # time.sleep(1)
    # limb.move([Position.X, Position.Y, 0.3], Orientation.DOWNWARDS)


if __name__ == '__main__':
    main()
