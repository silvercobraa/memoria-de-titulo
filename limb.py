#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import tf
import rospy
from pprint import pprint
import baxter_interface
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped
from geometry import Position, Orientation

class Limb():
    """Clase que abstrae un brazo con su gripper y cámara. """
    def __init__(self, side):
        if side != 'left' and side != 'right':
            raise Exception('Argumento inválido, debe ser "left" o "right".')
        self.side = side
        self._limb = baxter_interface.Limb(side)
        self._gripper = baxter_interface.Gripper(side)
        self._limb.set_joint_position_speed(0.9) #Sets the speed for the args arm. {0.0, 1.0}
        self.MAX_ANGLE = 3.059 # convenientemente es el mismo para w0 y w2, que son los que ocupo
        self.MIN_ANGLE = -3.059


    def move(self, position, orientation, recursive=False):
        """Adaptado del código de Julio."""
        service_name = '/ExternalTools/' + self.side + '/PositionKinematicsNode/IKService'
        ik_service = rospy.ServiceProxy(service_name, SolvePositionIK)
        frame = 'base'

        # self._limb.set_joint_position_speed(0.5)
        rospy.wait_for_service(service_name, 30)
        ik_message = SolvePositionIKRequest()
        ik_message.pose_stamp.append(self._make_pose_stamped(position, orientation))

        try:
            response = ik_service(ik_message)
        except:
            if recursive:
                self.fix()
                self.move(position, orientation, recursive=False)
            else:
                raise Exception("Excepción de ik_service.")

        if response.isValid[0] == True:
            movimiento = dict(zip(response.joints[0].name, response.joints[0].position))
            self._limb.move_to_joint_positions(movimiento)
        else:
            if recursive:
                self.fix()
                self.move(position, orientation, recursive=False)
            else:
                raise Exception("Respuesta inválida de ik_service.")


    def fix(self):
        ang = self._limb.joint_angles()[self.side + '_w1']
        self.set_angle('w1', 0.000)
        self.rotate('w0', 180)
        self.set_angle('w1', -ang)


    def _make_pose_stamped(self, position, orientation):
        t = PoseStamped()
        t.header.frame_id = 'base'
        t.header.stamp = rospy.Time.now()
        quaternion = tf.transformations.quaternion_from_euler(*orientation)
        t.pose.position.x, t.pose.position.y, t.pose.position.z = position
        t.pose.orientation.x, t.pose.orientation.y, t.pose.orientation.z, t.pose.orientation.w = quaternion
        return t



    def open(self):
        """Abre el gripper de este brazo."""
        self._gripper.open()


    def close(self):
        """Cierra el gripper de este brazo."""
        self._gripper.close()


    def calibrate(self):
        self._gripper.calibrate()


    def rotate_wrist(self, degrees):
        """Rota la muñeca de este brazo en 'degrees' grados. Útil para realizar los movimientos de las caras del cubo una vez se tenga agarrado."""
        # Ver este link para los limites de angulo del robot
        # http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications
        MAX_ANGLE = 3.059
        MIN_ANGLE = -3.059
        radians = 2*math.pi*degrees / 360
        angles = self._limb.joint_angles()
        wrist = self.side + '_w2'
        angles[wrist] += radians
        if angles[wrist] > max_angle:
            angles[wrist] -= 2*math.pi
        if angles[wrist] < MIN_ANGLE:
            # cambiar por warning?
            raise Exception('Angulo imposible:', angles[wrist])

        self._limb.move_to_joint_positions(angles)
        return self._limb.joint_angles()[wrist]


    def set_angle(self, joint, radians):
        """Setea la articulación 'joint' a 'radians' radianes. El string joint no incluye el prefijo del brazo. Ejemplo: 's0', 'e1', 'w2', etc. """
        # Ver este link para los limites de angulo del robot
        # http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications
        angles = self._limb.joint_angles()
        angles[self.side + '_' + joint] = radians
        ret = self._limb.move_to_joint_positions(angles)
        return ret

    def rotate(self, joint, degrees):
        """Setea la articulación 'joint' a 'radians' radianes. El string joint no incluye el prefijo del brazo. Ejemplo: 's0', 'e1', 'w2', etc. """
        # Ver este link para los limites de angulo del robot
        # http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications
        angles = self._limb.joint_angles()
        radians = 2*math.pi*degrees / 360
        joint_name = self.side + '_' + joint
        angles[joint_name] += radians
        if angles[joint_name] > self.MAX_ANGLE:
            angles[joint_name] -= 2*math.pi
        if angles[joint_name] < self.MIN_ANGLE:
            raise Exception('Angulo imposible:', angles[joint_name])

        ret = self._limb.move_to_joint_positions(angles)
        return ret



def main():
    import rospy
    import time
    rospy.init_node('limb')
    # limb = Limb('right', calibrate=True)
    limb = Limb('left')
    # descomentar para testear funciones
    # limb.close()
    # limb.open()
    # limb.rotate_wrist(90)
    # limb.rotate_wrist(180)
    # limb.rotate_wrist(270)
    # limb.set_angle('w1', 0)
    # limb.rotate('w0', 180)
    # limb.move(Position.ABOVE, Orientation.RIGHTWARDS_0)
    # limb.move(Position.ABOVE, Orientation.UPWARDS_90, True)
    DELTA = -0.15
    limb.move([Position.ABOVE[0], Position.ABOVE[1] + DELTA, Position.ABOVE[2]], Orientation.RIGHTWARDS_270)


if __name__ == '__main__':
    main()
