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
RIGHTWARDS2 = [-math.pi/2, -math.pi/2, -math.pi]
UPWARDS = [0, 0, -math.pi/2]
LEFTWARDS_0 = [math.pi/2, 0, math.pi]
LEFTWARDS_90 = [math.pi/2, math.pi/2, math.pi]
LEFTWARDS_180 = [math.pi/2, math.pi, math.pi]
LEFTWARDS_270 = [math.pi/2, -math.pi/2, math.pi]
DELTA = 0.1

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


def D90():
	right_grip.open()
	left_arm.move_baxter('base', MIDDLE, RIGHTWARDS)
	right_arm.move_baxter('base', [X, Y - DELTA, Z], LEFTWARDS_0)
	right_arm.move_baxter('base', [X, Y - 0.02, Z], LEFTWARDS_0)

	right_grip.close()
	time.sleep(1)
	right_arm.rotate_wrist(90)
	right_grip.open()
	time.sleep(1)
	right_arm.move_baxter('base', [X, Y - DELTA, Z], LEFTWARDS_270)

def D180():
	right_grip.open()
	left_arm.move_baxter('base', MIDDLE, RIGHTWARDS)
	right_arm.move_baxter('base', [X, Y - DELTA, Z], LEFTWARDS_0)
	right_arm.move_baxter('base', [X, Y - 0.02, Z], LEFTWARDS_0)

	right_grip.close()
	time.sleep(1)
	right_arm.rotate_wrist(180)
	right_grip.open()
	time.sleep(1)
	right_arm.move_baxter('base', [X, Y - DELTA, Z], LEFTWARDS_180)

def D270():
	right_grip.open()
	left_arm.move_baxter('base', MIDDLE, RIGHTWARDS)
	right_arm.move_baxter('base', [X, Y - DELTA, Z], LEFTWARDS_0)
	right_arm.move_baxter('base', [X, Y - 0.02, Z], LEFTWARDS_0)

	right_grip.close()
	time.sleep(1)
	right_arm.rotate_wrist(270)
	right_grip.open()
	time.sleep(1)
	right_arm.move_baxter('base', [X, Y - DELTA, Z], LEFTWARDS_90)



def switch_hands():
	right_grip.close()
	time.sleep(1)
	left_grip.open()
	time.sleep(1)
	right_arm.move_baxter('base', [X, Y + 0.01, Z], LEFTWARDS_0)

def main():

	# left_pickup()
	D90()
	D180()
	D270()
	return

	left_arm.move_baxter('base', [ABOVE[0] + 0.1, ABOVE[1], ABOVE[2]], UPWARDS)
	right_arm.move_baxter('base', [ABOVE[0] + 0.1, ABOVE[1] - 0.1, ABOVE[2]], LEFTWARDS_0)
	time.sleep(1)
	right_arm.move_baxter('base', [ABOVE[0] + 0.1, ABOVE[1] - 0.01, ABOVE[2]], LEFTWARDS_0)
	right_grip.close()
	time.sleep(1)
	right_arm.rotate_wrist(90)
	right_grip.open()
	time.sleep(1)
	right_arm.rotate_wrist(-90)
	time.sleep(1)
	right_arm.move_baxter('base', [ABOVE[0] + 0.1, ABOVE[1] - 0.01, ABOVE[2]], LEFTWARDS_0)
	left_arm.rotate_wrist(180)
	right_arm.move_baxter('base', [ABOVE[0] + 0.1, ABOVE[1] - 0.01, ABOVE[2]], LEFTWARDS_0)
	right_grip.close()
	time.sleep(1)
	right_arm.rotate_wrist(-90)
	right_grip.open()
	time.sleep(1)

	return
	right_grip.open()
	right_arm.move_baxter('base', [X, Y - 0.2, Z], LEFTWARDS_0)
	right_arm.move_baxter('base', [X, Y - 0.02, Z], LEFTWARDS_0)

	DEGREES = 180
	right_grip.close()
	time.sleep(1)
	right_arm.rotate_wrist(DEGREES)
	right_grip.open()
	time.sleep(1)
	left_grip.open()
	return

	right_arm.move_baxter('base', [X, Y - 0.01, Z], LEFTWARDS_0)
	# right_arm.move_baxter('base', [X, Y - 0.01, Z], LEFTWARDS_90)
	left_arm.move_baxter('base', MIDDLE, RIGHTWARDS2)
	right_grip.open()
	time.sleep(1)
	move_away(right_arm)
	left_grip.open()


if __name__ == "__main__":
	main()
