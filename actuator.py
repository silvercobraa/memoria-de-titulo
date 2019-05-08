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
        # self._holder = self._left # TODO: eliminar esta línea cuando el cubo esté en la mesa


    def U(self, degrees):
        """Rota la cara superior del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._right, Position.ABOVE, Orientation.LEFTWARDS_0, +0.015, +0.15)


    def R(self, degrees):
        """Rota la cara derecha del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._right, Position.ABOVE, Orientation.UPWARDS_0, +0.015, +0.15)


    def F(self, degrees):
        """Rota la cara frontal del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._left, Position.ABOVE, Orientation.UPWARDS_180, -0.015, -0.15)


    def D(self, degrees):
        """Rota la cara inferior del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._left, Position.ABOVE, Orientation.RIGHTWARDS_0, -0.015, -0.15)


    def L(self, degrees):
        """Rota la cara izquierda del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._right, Position.ABOVE, Orientation.UPWARDS_180, +0.015, +0.15)


    def B(self, degrees):
        """Rota la cara posterior del cubo en 'degrees' grados."""
        self._generic_move(degrees, self._left, Position.ABOVE, Orientation.UPWARDS_0, -0.015, -0.15)


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

        away_left = {
            90: Orientation.RIGHTWARDS_270,
            180: Orientation.RIGHTWARDS_180,
            270: Orientation.RIGHTWARDS_90,
        }
        away_right = {
            90: Orientation.LEFTWARDS_90,
            180: Orientation.LEFTWARDS_180,
            270: Orientation.LEFTWARDS_270,
        }
    	free_limb.move(free_limb_position_far, away_left[degrees] if self._holder == self._right else away_right[degrees])
    	# limb.move((position[0], position[1] - DELTA, position[2]), Orientation.LEFTWARDS_0 if limb == self._right else Orientation.RIGHTWARDS_0)


    def pick_up(self, side):
        """Recoge el cubo con la mano izquierda."""
        if self._holder is not None:
            raise Error('WARNING: el cubo fue recogido.')

        limb = self._right if side == 'right' else self._left
        self._holder = limb
        limb.open()
        limb.move(Position.MIDDLE2, Orientation.DOWNWARDS)
        limb.move([0.5, Position.Y, 0], Orientation.DOWNWARDS)
        limb.move(Position.CUBE_POSITION, Orientation.DOWNWARDS)
        limb.close()
    	time.sleep(1)
    	limb.move([Position.X, Position.Y, 0.3], Orientation.DOWNWARDS)


    def drop(self):
        """Deja el cubo."""
        self._holder.move(Position.FRONT, Orientation.FRONTWARDS_0)
        self._holder.open()


    def move(self, moves):
        """Realiza la secuencia de acciones descrita en el string 'moves'. El string consiste de letras y numeros"""
        for move in moves.split():
            self._check_hands(move[0])
            action = getattr(self, move[0].upper())
            degrees = 90 * int(move[1])
            print(move, degrees)
            # action(degrees)

    def switch_l2r():
    	self._left.move(Position.ABOVE, Orientation.RIGHTWARDS_0)
    	self._left.rotate_wrist(90)
    	self._right.move([Position.ABOVE[0], Position.ABOVE[1] - DELTA, Position.ABOVE[2]], Orientation.LEFTWARDS_0)
    	self._right.move([Position.ABOVE[0], Position.ABOVE[1] + 0.01, Position.ABOVE[2]], Orientation.LEFTWARDS_0)
    	right_grip.close()
    	time.sleep(1)
    	left_grip.open()
    	time.sleep(1)
    	self._left.move([Position.ABOVE[0], Position.ABOVE[1] + DELTA, Position.ABOVE[2]], away_left[90])

    def _check_hands(self, move):
        """Si el robot tiene el cubo en la mano izquierda, se lo pasa a la mano derecha. Si el robot tiene el cubo en la mano derecha, se lo pasa a la mano izquierda."""
        if self._holder == self._left and move in ['U', 'R', 'L']:
            print('switch cube from left to right hand')
        elif self._holder == self._right and move in ['D', 'F', 'B']:
            print('switch cube from right to left hand')



def main():
    rospy.init_node('main')
    actuator = Actuator(calibrate=True)
    # actuator.pick_up('left')
    actuator.D(90)

if __name__ == '__main__':
    main()
