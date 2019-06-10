import rospy
import os
from actuator import Limb, Actuator
from geometry import Position, Orientation
from color import ColorExtractor
from capturer import Capturer
import solver.solver as Solver
from classifier import Classifier

def main():
    DIR = 'capturas/'
    rospy.init_node('main')

    actuator = Actuator()
    # actuator = Actuator('right')
    # actuator = Actuator('left')
    actuator.set_capturer(Capturer(DIR, 1))

    actuator.calibrate()
    actuator.pick_up('left')
    # actuator.switch_l2r()
    # actuator.switch_r2l()
    # return

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

    # actuator.move(moves)

    actuator.drop()

if __name__ == '__main__':
    main()
