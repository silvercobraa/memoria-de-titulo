# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

# initialize our ROS node, registering it with the Master
rospy.init_node('Hello_Baxter')

# create an instance of baxter_interface's Limb class
left_limb = baxter_interface.Limb('left')
right_limb = baxter_interface.Limb('right')

# get the right limb's current joint angles
left_angles = left_limb.joint_angles()
right_angles = right_limb.joint_angles()

# print the current joint angles
print left_angles
print right_angles

# reassign new joint angles (all zeros) which we will later command to the limb
left_angles['left_s0']=0.0
left_angles['left_s1']=-0.5
left_angles['left_e0']=0.0
left_angles['left_e1']=0.0
left_angles['left_w0']=0.0
left_angles['left_w1']=0.0
left_angles['left_w2']=0.0


right_angles['right_s0']=0.0
right_angles['right_s1']=-0.5
right_angles['right_e0']=0.0
right_angles['right_e1']=0.0
right_angles['right_w0']=0.0
right_angles['right_w1']=0.0
right_angles['right_w2']=0.0

# move the right arm to those joint angles
left_limb.move_to_joint_positions(left_angles)
right_limb.move_to_joint_positions(right_angles)
