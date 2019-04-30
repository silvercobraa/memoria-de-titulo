#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import tf
import rospy
from pprint import pprint
import baxter_interface
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped

class Limb():
    """Clase que abstrae un brazo con su gripper y cámara. """
    def __init__(self, side, calibrate=False):
        if side != 'left' and side != 'right':
            raise Exception('Argumento inválido, debe ser "left" o "right".')
        self.side = side
        self._limb = baxter_interface.Limb(side)
        self._gripper = baxter_interface.Gripper(side)
        if calibrate: self._gripper.calibrate()

    def move(self, position, orientation):
        """Adaptado del código de Julio."""
        trans = position
        rot = orientation
        service_name = '/ExternalTools/' + self.side + '/PositionKinematicsNode/IKService'
        ik_service = rospy.ServiceProxy(service_name, SolvePositionIK)
        frame = 'base'

        self._limb.set_joint_position_speed(0.5)

        matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans),
            tf.transformations.euler_matrix(rot[0], rot[1], rot[2]))

        rospy.wait_for_service(service_name, 10)
        ik_message = SolvePositionIKRequest()
        ik_message.pose_stamp.append(self._message_matrix_to_pose(matrix, frame))

        try:
            response = ik_service(ik_message)
        except:
            print "Movement couldn't be executed"

        print response.isValid[0]

        if response.isValid[0] == True:
            movimiento = dict(zip(response.joints[0].name, response.joints[0].position))
            self._limb.move_to_joint_positions(movimiento)
        else:
            print "Movement couldn't be executed"

        print response.joints[0].position
        print response.joints[0].name

        pass


    def open(self):
        """Abre el gripper de este brazo."""
        self._gripper.open()


    def close(self):
        """Cierra el gripper de este brazo."""
        self._gripper.close()


    def rotate_wrist(self, degrees):
        """Rota la muñeca de este brazo en 'degrees' grados. Útil para realizar los movimientos de las caras del cubo una vez se tenga agarrado."""
        # Ver este link para los limites de angulo del robot
        # http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications
        max_angle = 3.059
        min_angle = -3.059
        radians = 2*math.pi*degrees / 360
        angles = self._limb.joint_angles()
        wrist = self.side + '_w2'
        pprint(angles)
        print 'radianes antes:', angles[wrist]
        angles[wrist] += radians
        print 'radianes despues:', angles[wrist]
        if angles[wrist] > max_angle:
            angles[wrist] -= 2*math.pi
        if angles[wrist] < min_angle:
            # cambiar por warning?
            raise Exception('Angulo imposible:', angles[wrist])

        pprint(angles)
        self._limb.move_to_joint_positions(angles)
        return self._limb.joint_angles()[wrist]


    def _message_matrix_to_pose(self, T, frame):
        t = PoseStamped()
        t.header.frame_id = frame
        t.header.stamp = rospy.Time.now()
        translation = tf.transformations.translation_from_matrix(T) #Transforms a matrix to x, y, z positions
        orientation = tf.transformations.quaternion_from_matrix(T)    #Transforms a matrix to x, y, z, w Quaternion angles
        t.pose.position.x = translation[0]
        t.pose.position.y = translation[1]
        t.pose.position.z = translation[2]
        t.pose.orientation.x = orientation[0]
        t.pose.orientation.y = orientation[1]
        t.pose.orientation.z = orientation[2]
        t.pose.orientation.w = orientation[3]
        return t


def main():
    import rospy
    import time
    rospy.init_node('limb')
    limb = Limb('right', calibrate=True)
    # limb = Limb('right', calibrate=False)
    # descomentar para testear funciones
    # limb.close()
    # limb.open()
    # limb.rotate_wrist(90)
    limb.rotate_wrist(180)
    # limb.rotate_wrist(270)


if __name__ == '__main__':
    main()
