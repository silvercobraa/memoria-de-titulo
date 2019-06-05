import rospy
from actuator import Limb, Actuator
from geometry import Position, Orientation
# from solver import Solver
# from color import Classifier

def main():
    rospy.init_node('main')
    actuator = Actuator()
    # actuator = Actuator('right')
    # actuator = Actuator('left')

    # actuator.calibrate()
    actuator.pick_up('left')

    actuator.capture()
    # actuator.R(90)
    # actuator.move('D1')
    # actuator.U(90)
    # actuator.L(90)
    # actuator.F(90)
    # actuator.D(90)
    # actuator.B(90)
    # actuator.switch_r2l()



if __name__ == '__main__':
    main()
