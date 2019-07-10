import rospy
import os
from actuator import Limb, Actuator
from geometry import Position, Orientation, Angle
from color import ColorExtractor
from capturer import Capturer
import solver.solver as Solver
from classifier import Classifier

def main():
    DIR = 'capturas/'
    rospy.init_node('main')

    actuator = Actuator()
    # actuator = Actuator('left')
    # actuator = Actuator('right')
    actuator.set_capturer(Capturer(DIR, 1))
    #
    actuator.calibrate()
    # actuator._right._limb.move_to_joint_positions(Angle.RIGHT_FAR)
    # actuator._left._limb.move_to_joint_positions(Angle.LEFT_FAR)
    actuator.pick_up('left')
    # actuator.switch_l2r()
    # actuator.switch_r2l()
    # actuator.move('F3 R1 D2 L1 U1 D3 B2 R2 F2 U3 D1 R2')
    # return
    #
    actuator.capture()

    ext = ColorExtractor(DIR, 'FRBLUD')
    colors = ext.get_representative_colors()
    ext.show()
    ext.save('reps')


    cls = Classifier('URFDLB')
    cls.fit(colors)
    state = cls.get_state()
    print(state)
    moves = Solver.solve(state, 20, 2)
    moves = moves[:moves.find('(')]
    print(moves)

    actuator.move(moves)
    # LRULRLURL
    # actuator._right._limb.move_to_joint_positions(Angle.L)
    # actuator._right._limb.move_to_joint_positions(Angle.R)
    # actuator._right._limb.move_to_joint_positions(Angle.U)
    # actuator._right._limb.move_to_joint_positions(Angle.L)
    # actuator._right._limb.move_to_joint_positions(Angle.R)
    # actuator._right._limb.move_to_joint_positions(Angle.L)
    # actuator._right._limb.move_to_joint_positions(Angle.U)
    # actuator._right._limb.move_to_joint_positions(Angle.R)
    # actuator._right._limb.move_to_joint_positions(Angle.L)
    # BDFBDBFDB
    # actuator._left._limb.move_to_joint_positions(Angle.B)
    # actuator._left._limb.move_to_joint_positions(Angle.D)
    # actuator._left._limb.move_to_joint_positions(Angle.F)
    # actuator._left._limb.move_to_joint_positions(Angle.B)
    # actuator._left._limb.move_to_joint_positions(Angle.D)
    # actuator._left._limb.move_to_joint_positions(Angle.B)
    # actuator._left._limb.move_to_joint_positions(Angle.F)
    # actuator._left._limb.move_to_joint_positions(Angle.D)
    # actuator._left._limb.move_to_joint_positions(Angle.B)

    actuator.drop()

if __name__ == '__main__':
    main()
