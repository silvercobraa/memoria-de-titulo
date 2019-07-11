#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time

import rospy
import baxter_interface as bi


def main():
    rospy.init_node('asdasd')
    D_angles = {
        'left_s0': math.pi/16,
        # 'left_s0': 0,
        'left_s1': 0,

        'left_e0': -math.pi/2,
        'left_e1': math.pi/2-0.09,

        'left_w0': 0,
        # 'left_w1': math.pi/3,
        'left_w1': 5*math.pi/16+0.09,
        # 'left_w1': math.pi/3,
        'left_w2': 0,
        'left_w2': 0,
        # 'left_w2': math.pi/2,
    }
    U_angles = {
        'right_s0': -math.pi/16,
        # 'right_s0': 0,
        'right_s1': 0,

        'right_e0': math.pi/2,
        'right_e1': math.pi/2-0.09,

        'right_w0': 0,
        # 'right_w1': math.pi/4,
        # 'right_w1': math.pi/3,
        'right_w1': 5*math.pi/16+0.09,
        'right_w2': 0,
    }
    F_angles = {
        'left_s0': -math.pi/12,
        'left_s1': 0,

        'left_e0': -math.pi/2,
        'left_e1': math.pi/2,

        'left_w0': -math.pi/2,
        'left_w1': math.pi/2,
        'left_w2': -math.pi/6,
    }
    B_angles = {
        'left_s0': -math.pi/12,
        'left_s1': 0,

        'left_e0': -math.pi/2,
        'left_e1': math.pi/2,

        'left_w0': -math.pi/2,
        'left_w1': math.pi/2,
        'left_w2': -math.pi/6 + math.pi,
    }
    R_angles = {
        'right_s0': math.pi/12,
        'right_s1': 0,

        'right_e0': math.pi/2,
        'right_e1': math.pi/2,

        'right_w0': math.pi/2,
        'right_w1': math.pi/2,
        'right_w2': math.pi/6,
    }
    L_angles = {
        'right_s0': math.pi/12,
        'right_s1': 0,

        'right_e0': math.pi/2,
        'right_e1': math.pi/2,

        'right_w0': math.pi/2,
        'right_w1': math.pi/2,
        'right_w2': math.pi/6 - math.pi,
    }
    l = bi.Limb('left')
    r = bi.Limb('right')
    lgrip = bi.Gripper('left')
    rgrip = bi.Gripper('right')
    lang = l.joint_angles()
    rang = r.joint_angles()
    print(lang)
    print(rang)
    l.move_to_joint_positions(D_angles)
    # l.move_to_joint_positions(F_angles)
    # l.move_to_joint_positions(B_angles)

    # r.move_to_joint_positions(R_angles)
    r.move_to_joint_positions(U_angles)
    # r.move_to_joint_positions(L_angles)
    while True:
        time.sleep(1)
        rgrip.close()
        time.sleep(1)
        rang = r.joint_angles()
        rang['right_w2'] += math.pi/2
        r.move_to_joint_positions(rang)
        time.sleep(1)
        rgrip.open()
        time.sleep(1)
        rang = r.joint_angles()
        rang['right_w1'] -= math.pi/2
        # r.move_to_joint_positions(rang)
        time.sleep(3)
        r.move_to_joint_positions(U_angles)



if __name__ == '__main__':
    main()
