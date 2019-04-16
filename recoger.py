import math
import rospy
import time
import baxter_interface

from basic import move

X = 0.5
Y = 0.0
Z = 0.2
CUBE_POSITION = [X, Y, -0.15]
MIDDLE = [X, Y, Z]
ABOVE = [X + 0.1, Y, 0.5]
DOWNWARDS = [-math.pi, 0, -math.pi]
RIGHTWARDS_0 = [-math.pi/2, 0, -math.pi]
RIGHTWARDS2 = [-math.pi/2, -math.pi/2, -math.pi]
RIGHTWARDS_90 = [-math.pi/2, math.pi/2, -math.pi]
RIGHTWARDS_180 = [-math.pi/2, math.pi, -math.pi]
RIGHTWARDS_270 = [-math.pi/2, -math.pi/2, -math.pi]
UPWARDS_0 = [0, 0, -math.pi/2]
UPWARDS_180 = [0, math.pi, math.pi/2]
LEFTWARDS_0 = [math.pi/2, 0, math.pi]
LEFTWARDS_90 = [math.pi/2, math.pi/2, math.pi]
LEFTWARDS_180 = [math.pi/2, math.pi, math.pi]
LEFTWARDS_270 = [math.pi/2, -math.pi/2, math.pi]
DELTA = 0.15

away_right = {
	90: LEFTWARDS_270,
	180: LEFTWARDS_180,
	270: LEFTWARDS_90,
}

away_left = {
	90: RIGHTWARDS_90,
	180: RIGHTWARDS_180,
	270: RIGHTWARDS_270,
}

rospy.init_node('move', anonymous = True) #Initializes a ros node
left_arm = move('left')
right_arm = move('right')
left_grip = baxter_interface.Gripper('left')
right_grip = baxter_interface.Gripper('right')

def descend(left_arm):
	left_arm.move_baxter('base', MIDDLE, DOWNWARDS)
	left_arm.move_baxter('base', [X, Y, 0], DOWNWARDS)
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
	left_arm.move_baxter('base', [X, Y, 0], DOWNWARDS)

def right_pickup():
	right_grip.open()
	descend(right_arm)
	time.sleep(1)
	right_grip.close()
	time.sleep(1)
	right_arm.move_baxter('base', [X, Y, 0], DOWNWARDS)


def D(degrees):
	right_grip.open()
	left_arm.move_baxter('base', MIDDLE, RIGHTWARDS_0)
	right_arm.move_baxter('base', [X, Y - DELTA, Z], LEFTWARDS_0)
	right_arm.move_baxter('base', [X, Y - 0.02, Z], LEFTWARDS_0)
	time.sleep(1)
	right_grip.close()
	time.sleep(1)
	right_arm.rotate_wrist(degrees)
	time.sleep(1)
	right_grip.open()
	time.sleep(1)
	right_arm.move_baxter('base', [X, Y - DELTA, Z], away_right[degrees])
	# right_arm.rotate_wrist(-90)


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


def F(degrees):
	right_grip.open()
	left_arm.move_baxter('base', ABOVE, UPWARDS_0)
	left_arm.rotate_wrist(180)
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


def U(degrees):
	left_grip.open()
	right_arm.move_baxter('base', MIDDLE, LEFTWARDS_0)
	left_arm.move_baxter('base', [X, Y + DELTA, Z], RIGHTWARDS_0)
	left_arm.move_baxter('base', [X, Y + 0.02, Z], RIGHTWARDS_0)

	time.sleep(1)
	left_grip.close()
	time.sleep(1)
	left_arm.rotate_wrist(degrees)
	time.sleep(1)
	left_grip.open()
	time.sleep(1)
	left_arm.move_baxter('base', [X, Y + DELTA, Z], away_left[degrees])


def switch_hands():
	right_grip.close()
	time.sleep(1)
	left_grip.open()
	time.sleep(1)
	right_arm.move_baxter('base', [X, Y + 0.01, Z], LEFTWARDS_0)


def left_drop():
	left_arm.move_baxter('base', [X + 0.5, Y, Z], [-math.pi/2, -math.pi/2, -math.pi/2])
	time.sleep(1)
	left_grip.open()


def main():
	# left_grip.calibrate()
	# right_grip.calibrate()
	# right_grip.open()
	# left_arm.move_baxter('base', MIDDLE, RIGHTWARDS_0)
	# F(180)
	# B(180)
	# D(180)
	# left_drop()

	# right_pickup()
	U(270)
	return


if __name__ == "__main__":
	main()
