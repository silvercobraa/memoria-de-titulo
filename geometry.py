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

    L_PICK_UP = {'left_w0': 1.3648594060210468,
    'left_w1': 1.9872721107055535,
    'left_w2': -1.5044516577186196,
    'left_e0': -2.0210196880390328,
    'left_e1': 1.8235196615987856,
    'left_s0': -0.4345000581685434,
    'left_s1': -0.0732475826215285}

    F_CAM = {'left_w0': -1.1378302494140056,
    'left_w1': 1.1048496624744693,
    'left_w2': -1.540883701430898,
    'left_e0': -1.7092380929013222,
    'left_e1': 2.1153595064939856,
    'left_s0': -0.388480634531981,
    'left_s1': 0.7727428218972772}

    B_CAM = {'left_w0': -1.131694326262464,
    'left_w1': 1.1102185952320682,
    'left_w2': 1.5715633171886063,
    'left_e0': -1.713456540068007,
    'left_e1': 2.1084565929485013,
    'left_s0': -0.39193209130472323,
    'left_s1': 0.768140879533621}

    # D_CAM = {'left_w0': -0.19558255045539025,
    # 'left_w1': 2.0674226068725665,
    # 'left_w2': 2.2572527293733864,
    # 'left_e0': -2.3650148797223367,
    # 'left_e1': 0.9272913862767326,
    # 'left_s0': -0.7551020428365949,
    # 'left_s1': 0.08283496254581234}

    D_CAM = {'left_w0': -0.6339175605936472,
    'left_w1': 2.0935002802666185,
    'left_w2': 0.8893253617765686,
    'left_e0': -1.905204138553684,
    'left_e1': 0.9621894492011258,
    'left_s0': -0.6469563972906732,
    'left_s1': -0.028762139772851508}

    R_CAM = {'right_s0': 0.3976845192592935,
    'right_s1': 0.8091748656095558,
    'right_w0': 1.1036991768835551,
    'right_w1': 1.0668836379743052,
    'right_w2': 1.5520050621430674,
    'right_e0': 1.7310973191286894,
    'right_e1': 2.121495429645527}

    L_CAM = {'right_s0': 0.432966077380658,
    'right_s1': 0.9487671173071284,
    'right_w0': 0.9825146946406075,
    'right_w1': 0.9476166317162144,
    'right_w2': -1.4829759266882236,
    'right_e0': 1.8277381087654705,
    'right_e1': 2.1425876654789517}

    # U_CAM = {'right_s0': 0.6638301859574128,
    # 'right_s1': 0.21552430069790063,
    # 'right_w0': 0.355883542789416,
    # 'right_w1': 1.9048206433567125,
    # 'right_w2': -0.7976700097004151,
    # 'right_e0': 2.1556265021759775,
    # 'right_e1': 1.1106020904290395}

    U_CAM = {'right_s0': 0.7892331153670453,
    'right_s1': 0.1062281695610649,
    'right_w0': 0.32060198466805145,
    'right_w1': 2.094267270660561,
    'right_w2': -0.7297913598364856,
    'right_e0': 2.2518837966157874,
    'right_e1': 0.7696748603215063}

    LEFT_CLOSE = {'left_w0': -0.1944320648644762,
    'left_w1': 0.8467573949127484,
    'left_w2': 0.30756314797102546,
    'left_e0': -1.3441506653845938,
    'left_e1': 1.8342575271139834,
    'left_s0': 0.32597091742565043,
    'left_s1': -0.04832039481839053}

    LEFT_FAR = {'left_w0': -0.11581554948534874,
    'left_w1': 1.5504710813551819,
    'left_w2': -0.13767477571271589,
    'left_e0': -1.6570827461132183,
    'left_e1': 1.084907912231959,
    'left_s0': 0.30871363356193954,
    'left_s1': -0.12156797743991904}

    LEFT_FAR_0 = {'left_w0': -0.11581554948534874,
    'left_w1': 0.0,
    'left_w2': -0.13767477571271589,
    'left_e0': -1.6570827461132183,
    'left_e1': 1.084907912231959,
    'left_s0': 0.30871363356193954,
    'left_s1': -0.12156797743991904}

    LEFT_FAR_2 = {'left_w0': -0.11581554948534874,
    'left_w1': 1.5504710813551819,
    'left_w2': -0.13767477571271589 + math.pi,
    'left_e0': -1.6570827461132183,
    'left_e1': 1.084907912231959,
    'left_s0': 0.30871363356193954,
    'left_s1': -0.12156797743991904}

    RIGHT_FAR = {'right_s0': -0.2972087776527989,
    'right_s1': 0.05368932757598948,
    'right_w0': -0.21283983431910117,
    'right_w1': 1.6375244910676792,
    'right_w2': -2.8627916453911535,
    'right_e0': 1.95505851415996,
    'right_e1': 1.031985575049912}

    RIGHT_FAR_0 = {'right_s0': -0.2972087776527989,
    'right_s1': 0.05368932757598948,
    'right_w0': -0.21283983431910117,
    'right_w1': 0.0,
    'right_w2': -2.8627916453911535,
    'right_e0': 1.95505851415996,
    'right_e1': 1.031985575049912}

    RIGHT_FAR_2 = {'right_s0': -0.2972087776527989,
    'right_s1': 0.05368932757598948,
    'right_w0': -0.21283983431910117,
    'right_w1': 1.6375244910676792,
    'right_w2': -2.8627916453911535 + math.pi,
    'right_e0': 1.95505851415996,
    'right_e1': 1.031985575049912}





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


def main():
    import rospy
    from limb import Limb
    rospy.init_node('asasdads')
    l = Limb('left')
    # l._limb.move_to_joint_positions(Angle.F_CAM)
    # l._limb.move_to_joint_positions(Angle.B_CAM)
    l._limb.move_to_joint_positions(Angle.D_CAM)

if __name__ == '__main__':
    main()
