import rospy
from actuator import Limb, Actuator
from geometry import Position, Orientation
# from solver import Solver
# from color import Classifier

def main():
    rospy.init_node('main')
    # actuator = Actuator()
    actuator = Actuator('right')
    # actuator = Actuator('left')

    # actuator.calibrate()
    # actuator.pick_up('left')

    # actuator.capture()
    actuator.move('B2 L2 F1 D2 F3 L2 D2 F3 U2 B1 D1 U3 L2 U1 F1 U3 F1 R3 F3 D2')
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
