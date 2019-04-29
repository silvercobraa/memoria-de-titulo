import math
import rospy
import time
import baxter_interface

from basic import move

X = 0.6
Y = 0.0
Z = 0.2
CUBE_POSITION = [0.5, 0.0, -0.15]
MIDDLE = [X, Y, Z]
MIDDLE2 = [0.5, Y, Z]
ABOVE = [X + 0.1, Y, 0.5]
DOWNWARDS = [-math.pi, 0, -math.pi]
RIGHTWARDS_0 = [math.pi/2, 0, 0]
RIGHTWARDS_90 = [math.pi/2, math.pi/2, 0]
RIGHTWARDS_180 = [math.pi/2, math.pi, 0]
RIGHTWARDS_270 = [math.pi/2, -math.pi/2, 0]
UPWARDS_0 = [0, 0, -math.pi/2]
UPWARDS_90 = [0, 0, 0]
UPWARDS_180 = [0, 0, math.pi/2]
UPWARDS_270 = [0, 0, math.pi]
LEFTWARDS_0 = [-math.pi/2, 0, 0]
LEFTWARDS_90 = [-math.pi/2, math.pi/2, 0]
LEFTWARDS_180 = [-math.pi/2, math.pi, 0]
LEFTWARDS_270 = [-math.pi/2, -math.pi/2, 0]
DELTA = 0.15

away_right = {
	90: LEFTWARDS_90,
	180: LEFTWARDS_180,
	270: LEFTWARDS_270,
}

away_left = {
	90: RIGHTWARDS_270,
	180: RIGHTWARDS_180,
	270: RIGHTWARDS_90,
}

rospy.init_node('move', anonymous = True) #Initializes a ros node
left_arm = move('left')
right_arm = move('right')
left_grip = baxter_interface.Gripper('left')
right_grip = baxter_interface.Gripper('right')

def descend(left_arm):
	left_arm.move_baxter('base', MIDDLE2, DOWNWARDS)
	left_arm.move_baxter('base', [0.5, Y, 0], DOWNWARDS)
	left_arm.move_baxter('base', CUBE_POSITION, DOWNWARDS)

def ascend(left_arm):
	left_arm.move_baxter('base', [X, Y, 0], DOWNWARDS)
	left_arm.move_baxter('base', MIDDLE, DOWNWARDS)

def move_away(right_arm):
	right_arm.move_baxter('base', [X, Y - 0.3, Z], LEFTWARDS_0)

def left_pickup():
	left_grip.open()
	descend(left_arm)
	left_grip.close()
	time.sleep(1)
	left_arm.move_baxter('base', [X, Y, 0.3], DOWNWARDS)

def right_pickup():
	right_grip.open()
	descend(right_arm)
	time.sleep(1)
	right_grip.close()
	time.sleep(1)
	right_arm.move_baxter('base', [X, Y, 0.3], DOWNWARDS)


def D(degrees):
	right_grip.open()
	left_arm.move_baxter('base', ABOVE, RIGHTWARDS_0)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - DELTA, ABOVE[2]], LEFTWARDS_0)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - 0.015, ABOVE[2]], LEFTWARDS_0)
	time.sleep(1)
	right_grip.close()
	time.sleep(1)
	right_arm.rotate_wrist(degrees)
	time.sleep(1)
	right_grip.open()
	time.sleep(1)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - DELTA, ABOVE[2]], away_right[degrees])
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + DELTA, ABOVE[2]], away_left[90])


def B(degrees):
	right_grip.open()
	left_arm.move_baxter('base', ABOVE, UPWARDS_0)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - DELTA, ABOVE[2]], LEFTWARDS_0)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - 0.01, ABOVE[2]], LEFTWARDS_0)

	time.sleep(1)
	right_grip.close()
	time.sleep(1)
	right_arm.rotate_wrist(degrees)
	time.sleep(1)
	right_grip.open()
	time.sleep(1)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - DELTA, ABOVE[2]], away_right[degrees])
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + DELTA, ABOVE[2]], away_left[90])


def F(degrees):
	right_grip.open()
	left_arm.move_baxter('base', ABOVE, UPWARDS_180)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - DELTA, ABOVE[2]], LEFTWARDS_0)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - 0.013, ABOVE[2]], LEFTWARDS_0)

	time.sleep(1)
	right_grip.close()
	time.sleep(1)
	right_arm.rotate_wrist(degrees)
	time.sleep(1)
	right_grip.open()
	time.sleep(1)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - DELTA, ABOVE[2]], away_right[degrees])
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + DELTA, ABOVE[2]], away_left[90])


def U(degrees):
	left_grip.open()
	right_arm.move_baxter('base', ABOVE, LEFTWARDS_0)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + DELTA, ABOVE[2]], RIGHTWARDS_0)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + 0.01, ABOVE[2]], RIGHTWARDS_0)

	time.sleep(1)
	left_grip.close()
	time.sleep(1)
	left_arm.rotate_wrist(degrees)
	time.sleep(1)
	left_grip.open()
	time.sleep(1)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + DELTA, ABOVE[2]], away_left[degrees])


# podria ser L...
def R(degrees):
	left_grip.open()
	right_arm.move_baxter('base', ABOVE, UPWARDS_0)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + DELTA, ABOVE[2]], RIGHTWARDS_0)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + 0.025, ABOVE[2]], RIGHTWARDS_0)
	#
	time.sleep(1)
	left_grip.close()
	time.sleep(1)
	left_arm.rotate_wrist(degrees)
	time.sleep(1)
	left_grip.open()
	time.sleep(1)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + DELTA, ABOVE[2]], away_left[degrees])


def L(degrees):
	left_grip.open()
	right_arm.move_baxter('base', ABOVE, UPWARDS_180)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + DELTA, ABOVE[2]], RIGHTWARDS_0)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + 0.017, ABOVE[2]], RIGHTWARDS_0)

	time.sleep(1)
	left_grip.close()
	time.sleep(1)
	left_arm.rotate_wrist(degrees)
	time.sleep(1)
	left_grip.open()
	time.sleep(1)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + DELTA, ABOVE[2]], away_left[degrees])


def switch_l2r():
	left_arm.move_baxter('base', ABOVE, RIGHTWARDS_0)
	left_arm.rotate_wrist(90)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - DELTA, ABOVE[2]], LEFTWARDS_0)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + 0.01, ABOVE[2]], LEFTWARDS_0)
	right_grip.close()
	time.sleep(1)
	left_grip.open()
	time.sleep(1)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + DELTA, ABOVE[2]], away_left[90])

def switch_r2l():
	right_arm.move_baxter('base', ABOVE, LEFTWARDS_0)
	right_arm.rotate_wrist(90)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] + DELTA, ABOVE[2]], RIGHTWARDS_0)
	left_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - 0.01, ABOVE[2]], RIGHTWARDS_0)
	left_grip.close()
	time.sleep(1)
	right_grip.open()
	time.sleep(1)
	right_arm.move_baxter('base', [ABOVE[0], ABOVE[1] - DELTA, ABOVE[2]], away_right[90])


def left_drop():
	left_arm.move_baxter('base', [X + 0.5, Y, Z], [-math.pi/2, -math.pi/2, -math.pi/2])
	time.sleep(1)
	left_grip.open()

def right_drop():
	right_arm.move_baxter('base', [X + 0.5, Y, Z], [-math.pi/2, -math.pi/2, -math.pi/2])
	time.sleep(1)
	right_grip.open()


def main():
	# left_pickup()
	D(270)
	B(180)
	F(90)
	B(180)
	D(270)
	F(90)
	D(270)
	# switch_l2r()
	# R(90)
	# L(90)
	# U(90)
	# right_drop()

	# left_arm.move_baxter('base', [ABOVE[0] - 0.2, ABOVE[1], ABOVE[2] + 0.1], UPWARDS_270)
	return


if __name__ == "__main__":
	main()
