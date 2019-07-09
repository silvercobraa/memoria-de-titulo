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

class Angle:
    """Angulos de posiciones favorables para los giros de cada cara."""

    def __init__(self):
        raise Error('Clase no istanciable.')

    D = {'left_e0': -2.53605373757156,
     'left_e1': 1.4684031092033123,
     'left_s0': -0.18254371375836423,
     'left_s1': 0.5161845351234418,
     'left_w0': 1.023165185519571,
     'left_w1': 1.3855681466575,
     'left_w2': 2.4071993513891856}

    F = {'left_w0': -1.240223467005357,
    'left_w1': 1.9017526817809416,
    'left_w2': -0.6902913545484362,
    'left_e0': -1.462650681248742,
    'left_e1': 0.979830228261808,
    'left_s0': -0.5817622138055432,
    'left_s1': 0.44293695250191323}

    B = {'left_w0': -1.0415729549741959,
    'left_w1': 1.8200682048260435,
    'left_w2': 2.334335263964628,
    'left_e0': -1.652480803749562,
    'left_e1': 0.971009838731467,
    'left_s0': -0.5852136705782853,
    'left_s1': 0.5691068723054885}

    L = {'right_s0': 0.6273981422451342,
    'right_s1': 0.7462816533062537,
    'right_w0': 0.744364177321397,
    'right_w1': 1.716524501643778,
    'right_w2': -2.298286715449321,
    'right_e0': 1.9634954084933296,
    'right_e1': 0.9844321706254643}

    R = {'right_s0': 0.5836796897904,
    'right_s1': 0.5740923098661161,
    'right_w0': 1.0262331470953419,
    'right_w1': 1.8123983008866162,
    'right_w2': 0.7432136917304829,
    'right_e0': 1.6662866308405306,
    'right_e1': 0.9813642090496935}

    U = {'right_s0': 0.0337475773334791,
    'right_s1': 0.3861796633501529,
    'right_w0': -0.795369038518587,
    'right_w1': 1.2555632748842112,
    'right_w2': -2.5191799489048208,
    'right_e0': 2.297136229858407,
    'right_e1': 1.462650681248742}


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
