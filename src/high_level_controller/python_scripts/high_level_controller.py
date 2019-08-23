#!/usr/bin/env python

import rospy
import time
import random
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

# global variables
global roll_go_home_flag
global roll_go_open_door_flag
global roll_move_flag
global gripper_go_home_flag
global gripper_go_open_door_flag
global gripper_move_flag
gripper_move_flag = 0 # whether we should be moving the gripper
gripper_go_home_flag = 0 # whichg direction to move gripper
gripper_go_open_door_flag = 0 # which direction to move gripper
roll_go_home_flag = 0 # which direction to move roll
roll_go_open_door_flag = 0 # which direction to move roll
roll_move_flag = 0

def joy_callback(data):
	global roll_go_home_flag
	global roll_go_open_door_flag
	global roll_move_flag
	global gripper_go_home_flag
	global gripper_go_open_door_flag
	global gripper_move_flag

	# check if pressing "A"
	if data.buttons[0] == 1:
		# basically a go signal for predetermined return to home position
		z_actuator_command_pub.publish(0)

	# check if pressing "B"
	if data.buttons[1] == 1:
		# basically a go signal for predetermined action primitive
		z_actuator_command_pub.publish(1)

	# check if pressing "X"
	if data.buttons[2] == 1:
		# basically a go signal for predetermined return to home position
		gripper_move_flag = 1
		gripper_go_home_flag = 1
		roll_go_home_flag = 1
		# roll_actuator_command_pub.publish(0)

	# check if pressing "Y"
	if data.buttons[3] == 1:
		# basically a go signal for predetermined action primitive
		gripper_move_flag = 1
		gripper_go_open_door_flag = 1 # have to delay move
		roll_go_open_door_flag = 1
		# roll_actuator_command_pub.publish(1)

	if data.buttons[5] == 1:
		# stop gripper
		gripper_command_pub.publish(2)


def main():
	# initialize node
	rospy.init_node('xbox2Arduino', anonymous = True)

	# initialize publishers
	global z_actuator_command_pub
	z_actuator_command_pub = rospy.Publisher('z_actuator_command', Float32, queue_size = 10)
	global roll_actuator_command_pub
	roll_actuator_command_pub = rospy.Publisher('roll_actuator_command', Float32, queue_size = 10)
	global gripper_command_pub
	gripper_command_pub = rospy.Publisher('gripper_command', Float32, queue_size = 10)

	# put both actuators to home positions
	#z_actuator_command_pub.publish(0)
	#roll_actuator_command_pub.publish(0)

	# initialize joy subscriber
	global joySubscriber
	joySubscriber = rospy.Subscriber("joy", Joy, joy_callback)

	# high level control loop
	control_loop()

	# rospy.spin()
	rospy.spin()


def control_loop():
	# send send send
	pubRate = 50 # hertz
	rate = rospy.Rate(pubRate) # hertz

	# gripper move variables
	gripper_move_time = 3 # how long to delay the gripper moving (seconds)
	gripper_move_counter = 0

	# global variables
	global roll_go_home_flag
	global roll_go_open_door_flag
	global roll_move_flag
	global gripper_go_home_flag
	global gripper_go_open_door_flag
	global gripper_move_flag

	while (not rospy.is_shutdown()):
		# move the gripper
		if gripper_move_flag == 1:
			print('moving gripper')
			if gripper_go_home_flag == 1:
				print('gripper going home')
				gripper_command_pub.publish(0)
			if gripper_go_open_door_flag == 1:
				print('gripper going to open door')
				gripper_command_pub.publish(1)

			# iterate the counter
			gripper_move_counter = gripper_move_counter + 1

		# check if the gripper should still be being moved
		if (gripper_move_counter >= gripper_move_time*pubRate) and (gripper_move_flag == 1):
			gripper_go_home_flag = 0
			gripper_go_open_door_flag = 0
			gripper_move_counter = 0
			gripper_move_flag = 0
			gripper_command_pub.publish(2) # stop command
			print('stopping gripper')

			# give permission to move roll
			roll_move_flag = 1

		# move the roll
		if roll_move_flag == 1:
			if roll_go_home_flag == 1:
				roll_actuator_command_pub.publish(0)
			if roll_go_open_door_flag == 1:
				roll_actuator_command_pub.publish(1)

			# reset everything
			roll_go_home_flag = 0
			roll_go_open_door_flag = 0
			roll_move_flag = 0

		# sleep to maintain loop rate
		rate.sleep()


def parse_force_glove():
	# eventually parse force glove and send action primitives go signals. for now...
	pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
