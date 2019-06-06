import rospy
import os
from actuator import Limb, Actuator
from geometry import Position, Orientation
from color import ColorExtractor
from capturer import Capturer
# from solver import Solver
# from color import Classifier

def main():
    DIR = 'capturas_temp/'
    rospy.init_node('main')

    # actuator = Actuator()
    # actuator = Actuator('right')
    actuator = Actuator('left')
    actuator.set_capturer(Capturer(DIR, 1))

    # actuator.calibrate()
    # actuator.pick_up('left')

    actuator.capture()
    # actuator.move('B2 L2 F1 D2 F3 L2 D2 F3 U2 B1 D1 U3 L2 U1 F1 U3 F1 R3 F3 D2')

    ext = ColorExtractor('capturas/', 'FRBLUD')
    colors = ext.get_representative_colors()
    ext.show()
    ext.save('reps')



if __name__ == '__main__':
    main()
