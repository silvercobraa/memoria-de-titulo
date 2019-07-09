#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

class Position():
    """Constantes para las posiciones. Cada posición es una tupla de 3 elementos, donde cada una corresponde a su posición en metros en los ejes (x, y, z) respectivamente. La orientación (0,0,0) corresponde a el gripper apuntando hacia arriba, en forma de 'U' visto desde el frente."""

    def __init__(self):
        raise Error('Clase no instanciable.')

    X = 0.6
    Y = 0.0
    Z = 0.2
    # CUBE_POSITION = (0.5, 0.0, -0.15)
    CUBE_POSITION = (0.5, 0.0, -0.16)
    MIDDLE = (X, Y, Z)
    MIDDLE2 = (0.5, Y, Z)
    ABOVE = (X + 0.1, Y, 0.5)
    ABOVE2 = (0.7, 0.0, 0.7)
    FRONT = (X + 0.5, Y, Z)
    # HEAD_CAMERA2 = (0.35, 0, 0.55) # FRENTE
    HEAD_CAMERA2 = (0.35, 0, 0.70) # FRENTE
    # HEAD_CAMERA = (0.5, 0, 0.75) # FRENTE
    HEAD_CAMERA = (0.5, 0, 0.70) # FRENTE


class Orientation():
    """Constantes para las orientaciones. Cada orientación es una tupla de 3 elementos, donde cada una corresponde a ángulos en radianes en los ejes (x, y, z) respectivamente. La orientación (0,0,0) corresponde a el gripper apuntando hacia arriba, en forma de 'U' visto desde el frente."""

    def __init__(self):
        raise Error('Clase no instanciable.')

    DOWNWARDS = (-math.pi, 0, -math.pi)
    RIGHTWARDS_0 = (math.pi/2, 0, 0)
    RIGHTWARDS_90 = (math.pi/2, math.pi/2, 0)
    RIGHTWARDS_180 = (math.pi/2, math.pi, 0)
    RIGHTWARDS_270 = (math.pi/2, -math.pi/2, 0)
    UPWARDS_0 = (0, 0, -math.pi/2)
    UPWARDS_90 = (0, 0, 0)
    UPWARDS_90_2 = (0, 0.2, 0)
    UPWARDS_180 = (0, 0, math.pi/2)
    UPWARDS_270 = (0, 0, math.pi)
    UPWARDS_270_2 = (0, -0.2, math.pi)
    LEFTWARDS_0 = (-math.pi/2, 0, 0)
    LEFTWARDS_90 = (-math.pi/2, math.pi/2, 0)
    LEFTWARDS_180 = (-math.pi/2, math.pi, 0)
    LEFTWARDS_270 = (-math.pi/2, -math.pi/2, 0)

    FRONTWARDS_0 = (0, math.pi/2, 0)
    BACKWARDS_0 = (0, -3*math.pi/8, 0)
    # BACKWARDS_0 = (0, -math.pi/2, 0)
