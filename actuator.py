#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import time

from geometry import Position, Orientation
from limb import Limb

class Actuator():
    """Clase que abstrae los actuadores del robot."""

    def __init__(self, calibrate=False):
        self._holder = None # la mano que tiene  actualmente el cubo
        self._left = Limb('left', calibrate)
        self._right = Limb('right', calibrate)


    def U(self, degrees):
        """Rota la cara superior del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._right, Position.ABOVE, Orientation.LEFTWARDS_0, +0.015, +0.15)
        pass


    def R(self, degrees):
        """Rota la cara derecha del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._right, Position.ABOVE, Orientation.UPWARDS_0, +0.015, +0.15)
        pass


    def F(self, degrees):
        """Rota la cara frontal del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._left, Position.ABOVE, Orientation.UPWARDS_180, -0.015, -0.15)
        pass


    def D(self, degrees):
        """Rota la cara inferior del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._left, Position.ABOVE, Orientation.RIGHTWARDS_0, -0.015, -0.15)


    def L(self, degrees):
        """Rota la cara izquierda del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._right, Position.ABOVE, Orientation.UPWARDS_180, +0.015, +0.15)
        pass


    def B(self, degrees):
        """Rota la cara posterior del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._left, Position.ABOVE, Orientation.UPWARDS_0, -0.015, -0.15)
        pass


    def _generic_move(self, degrees, limb, position, orientation, delta, DELTA):
        free_limb = self._left if limb == self._right else self._right
        free_limb_position_close = (position[0], position[1] + delta, position[2])
        free_limb_position_far = (position[0], position[1] + DELTA, position[2])
        free_limb_orientation = Orientation.RIGHTWARDS_0 if limb == self._right else Orientation.LEFTWARDS_0
        free_limb.open()
    	limb.move(position, orientation)
    	free_limb.move(free_limb_position_far, free_limb_orientation)
    	free_limb.move(free_limb_position_close, free_limb_orientation)
    	time.sleep(1)
    	free_limb.close()
    	time.sleep(1)
    	free_limb.rotate_wrist(degrees)
    	time.sleep(1)
    	free_limb.open()
    	time.sleep(1)
    	# free_limb.move(free_limb_position_far, away_right[degrees])
    	# limb.move([position[0], position[1] + DELTA, position[2]], away_left[90])


    def pick_up(self, side):
        """Recoge el cubo con la mano izquierda."""
        if self._holder is not None:
            raise Error('WARNING: el cubo fue recogido.')

        self._holder = side
        limb = self._right if side == 'right' else self._left
        limb.open()
        limb.move(Position.MIDDLE2, Orientation.DOWNWARDS)
        limb.move([0.5, Position.Y, 0], Orientation.DOWNWARDS)
        limb.move(Position.CUBE_POSITION, Orientation.DOWNWARDS)
        limb.close()
    	time.sleep(1)
    	limb.move([Position.X, Position.Y, 0.3], Orientation.DOWNWARDS)


    def drop():
        """Deja el cubo."""
        pass


    def move(moves):
        """Realiza la secuencia de acciones descrita en el string 'moves'. El string consiste de letras y numeros"""
        pass


    def _check_hands():
        """Si el robot tiene el cubo en la mano izquierda, se lo pasa a la mano derecha. Si el robot tiene el cubo en la mano derecha, se lo pasa a la mano izquierda."""
        if self._holder == 'left':
            pass
        elif self._holder == 'right':
            pass
        pass


def main():
    rospy.init_node('main')
    actuator = Actuator(calibrate=True)
    actuator.pick_up('left')

if __name__ == '__main__':
    main()
