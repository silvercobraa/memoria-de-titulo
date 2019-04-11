import math
import rospy
import time
import baxter_interface

from basic import move

X = 0.5
Y = 0.0
Z = 0.3
CUBE_POSITION = [X, Y, -0.15]
MIDDLE = [X, Y, Z]
ABOVE = [X, Y, 0.5]
DOWNWARDS = [-math.pi, 0, -math.pi]
RIGHTWARDS = [-math.pi/2, 0, -math.pi]
UPWARDS = [0, 0, -math.pi/2]
LEFTWARDS = [math.pi/2, 0, math.pi]
LEFTWARDS2 = [math.pi/2, math.pi/2, math.pi]

def descend(left_arm):
	left_arm.move_baxter('base', MIDDLE, DOWNWARDS)
	left_arm.move_baxter('base', [X, Y, 0], DOWNWARDS)
	left_arm.move_baxter('base', CUBE_POSITION, DOWNWARDS)

def ascend(left_arm):
	left_arm.move_baxter('base', [X, Y, 0], DOWNWARDS)
	left_arm.move_baxter('base', MIDDLE, DOWNWARDS)

def move_away(right_arm):
	right_arm.move_baxter('base', [X, Y - 0.3, Z], LEFTWARDS)


def main():

	rospy.init_node('move', anonymous = True) #Initializes a ros node
	left_arm = move('left')
	right_arm = move('right')
	left_grip = baxter_interface.Gripper('left')
	right_grip = baxter_interface.Gripper('right')

	# move_away(right_arm)
	# descend(left_arm)
	# left_grip.open()
	# ascend(left_arm)
	left_arm.move_baxter('base', ABOVE, UPWARDS)
	# right_arm.move_baxter('base', [X, Y, 0.55], LEFTWARDS2)
	return

	left_grip.open()
	descend(left_arm)
	left_grip.close()
	time.sleep(1)
	left_arm.move_baxter('base', [X, Y, 0], DOWNWARDS)
	left_arm.move_baxter('base', MIDDLE, RIGHTWARDS)

	right_grip.open()
	right_arm.move_baxter('base', [X, Y - 0.2, Z], LEFTWARDS)
	right_arm.move_baxter('base', [X, Y - 0.01, Z], LEFTWARDS)
	right_grip.close()
	time.sleep(1)
	right_arm.move_baxter('base', [X, Y - 0.01, Z], LEFTWARDS)
	right_arm.move_baxter('base', [X, Y - 0.01, Z], LEFTWARDS2)
	right_grip.open()
	time.sleep(1)
	move_away(right_arm)
	# left_grip.open()


if __name__ == "__main__":
	main()
