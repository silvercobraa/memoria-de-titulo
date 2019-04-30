import rospy
from actuator import Limb, Actuator
from geometry import Position, Orientation
# from solver import Solver
# from color import Classifier

def main():
    rospy.init_node('main')
    actuator = Actuator()
    # actuator.pick_up('right')
    actuator.U(90)


if __name__ == '__main__':
    main()
