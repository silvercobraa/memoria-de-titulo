import rospy
import os
import time
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
    actuator.pick_up('left')
    # actuator.switch_l2r()
    # actuator.switch_r2l()
    # actuator.move('R2 L1 F1 U2 F2 R3 F3 B3 R3 L3 U3 D1')
    # time.sleep(1)
    # return
    #
    actuator.capture()

    ext = ColorExtractor(DIR, 'FRBLUD')
    colors = ext.get_representative_colors()
    ext.show()
    ext.save('reps')

    cls = Classifier('URFDLB')
    cls.fit3(colors)
    state = cls.get_state()
    print(state)
    moves = Solver.solve(state, 20, 2)
    moves = moves[:moves.find('(')]
    print(moves)

    actuator.move(moves)
    # LRULRLURL
    # BDFBDBFDB

    actuator.drop()

if __name__ == '__main__':
    main()
