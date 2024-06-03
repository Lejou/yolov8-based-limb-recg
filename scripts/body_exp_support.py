#!/usr/bin/env python3

from __future__ import print_function
from asyncio import current_task

import copy
from email import header
import rospy
import PandaMassage
from time import sleep
import numpy
import geometry_msgs.msg

def main():

	pneumatic = PandaMassage.ArduinoPneumaticActuator()

	home_point = [0.5286889490021094, -0.1389050686968094, 0.3252420799100697, 0.9998611758779029, 0.012269542854380361, -0.011253558895930662, 0.0007079367156619242]
	pneumatic.panda.set_start_point(home_point)
 
	wpose = pneumatic.panda.read_current_pose()

	target_waypoints_record=numpy.array([
	[0.5520645522786755, -0.0063758257746355015, 0.14010357965771825, 0.9890577869544649, -0.1130690734605104, -0.03749231416929212, 0.08702786226549612] ,

	[0.5163440243861737, -0.13982390415589047, 0.1741247651257281, 0.9873867799623379, -0.09945247837600035, 0.010576709911068141, 0.12273682510994718] 
	])


	# target rotation eular angles referred to world frame, unit degree
	rotation_array=numpy.array([
		[5,0,0],
		[-5,0,0],
		[0,10,0],
		[0,5,0],
		[0,-5,0],
		[0,-10,0],
		[0,0,10],
		[0,0,5],
		[0,0,-5],
		[0,0,-10]
	])
 
	position_compensate_array=numpy.array([
		[0,0,0],
		[0,0,0],
		[0.02,0,0],
		[0.01,0,0],
		[-0.01,0,0],
		[-0.02,0,0],
		[0,0,0],
		[0,0,0],
		[0,0,0],
		[0,0,0]
	])
	# print("target_waypoints_record ", target_waypoints_record)

	(row,col) = target_waypoints_record.shape
	(rotation_row,col) = rotation_array.shape
	wpose = geometry_msgs.msg.Pose()
	# wpose = pneumatic.panda.move_group.get_current_pose().pose
	target_waypoints = []
	interval_time_ =4

	for i in range(row):
		wpose.position.x = target_waypoints_record[i,0]
		wpose.position.y = target_waypoints_record[i,1]
		wpose.position.z = target_waypoints_record[i,2]
		wpose.orientation.x=target_waypoints_record[i,3]
		wpose.orientation.y=target_waypoints_record[i,4]
		wpose.orientation.z=target_waypoints_record[i,5]
		wpose.orientation.w=target_waypoints_record[i,6]
		target_waypoints.append(copy.deepcopy(wpose))


	head = input("============ Choose the head, general:1;bio:2...")
 
	if head == '1':
		# # general head
		ready_depth = -0.04
		NP_depth = 0.026
	else:
		# bio_inspired head
		ready_depth = -0.04
		NP_depth = 0.02
 

 
	try:
		print("Start the massage!")
		input("============ Press `Enter` to move to the home joint ...")
		pneumatic.turn_off_main_pressure()
		pneumatic.panda.set_joint_velocity_scale(0.2)
		pneumatic.panda.set_joint_acceleration_scale(0.2)
		# pneumatic.panda.move_to_start()
		# pneumatic.panda.go_to_home_joint()



		# input("============ Press `Enter` to move to the start point and turn on cycle pressure ...")
		pneumatic.panda.move_to_start()
  
		pneumatic.turn_on_cycle_pressure(225,interval_time_)
		pneumatic.panda.set_joint_velocity_scale(0.2)
		pneumatic.panda.set_joint_acceleration_scale(0.1)
		print("Record data!")
		pneumatic.turn_on_recording()
  
		mode = input("============ Press `Enter` to massage ...")
		if mode == '':
			for row_i in range(0,row):
				row_success_flag_cnt =0
			# for row_i in range(3,6):
				print("============ massage target pose ",row_i+1)
				# input("")
				row_success_flag = pneumatic.excute_cycle_massage(target_waypoints[row_i], interval_time_,ready_depth,NP_depth)
				if row_success_flag:
					row_success_flag_cnt = row_success_flag_cnt+1
				# print("row_success_flag : ",row_success_flag)
				for rotation_row_i in range(0,rotation_row):
					print("============ massage target pose rotation ",rotation_array[rotation_row_i])
					# input("")
					# new_pose = pneumatic.new_pose_after_eular_rotation(0,30,0,target_waypoints[row_i])
					# new_pose = pneumatic.new_pose_after_eular_array_rotation(rotation_array[rotation_row_i],target_waypoints[row_i])
					pose_from_array = copy.deepcopy(target_waypoints[row_i])
					new_pose = pneumatic.new_pose_from_array_rotation(position_compensate_array[rotation_row_i],rotation_array[rotation_row_i],pose_from_array)
					row_success_flag = pneumatic.excute_cycle_massage(new_pose, interval_time_,ready_depth,NP_depth)
					# pneumatic.excute_cycle_massage(new_pose, interval_time_,ready_depth,NP_depth)
					if row_success_flag:
						row_success_flag_cnt = row_success_flag_cnt+1	
					# print("row_success_flag : ",row_success_flag)
				print("********************************* massage target pose success percentage ",row_success_flag_cnt)
		elif mode == 't':
			for row_i in range(0,row):
				print("============ massage target pose ",row_i+1)
				# input("")
				row_success_flag = pneumatic.excute_cycle_massage(target_waypoints[row_i], interval_time_,ready_depth,NP_depth)
		else:
			c = 'c'
			while 1:
				c = input("============ input step massage num ...")
				if c == '':	
					break
				num = ord(c)-ord('0')
				print("num :" ,num)
				if num < row+1:
					row_success_flag_cnt =0
				# for row_i in range(3,6):
					print("============ massage target pose ",num-1)
					# input("")
					row_success_flag = pneumatic.excute_cycle_massage(target_waypoints[num-1], interval_time_,ready_depth,NP_depth)
					if row_success_flag:
						row_success_flag_cnt = row_success_flag_cnt+1	
					for rotation_row_i in range(0,rotation_row):
						print("============ massage target pose rotation ",rotation_array[rotation_row_i])
						# input("")
						# new_pose = pneumatic.new_pose_after_eular_rotation(0,30,0,target_waypoints[row_i])
						# new_pose = pneumatic.new_pose_after_eular_array_rotation(rotation_array[rotation_row_i],target_waypoints[row_i])
						pose_from_array = copy.deepcopy(target_waypoints[num-1])
						new_pose = pneumatic.new_pose_from_array_rotation(position_compensate_array[rotation_row_i],rotation_array[rotation_row_i],pose_from_array)
						row_success_flag = pneumatic.excute_cycle_massage(new_pose, interval_time_,ready_depth,NP_depth)
						if row_success_flag:
							row_success_flag_cnt = row_success_flag_cnt+1	
						print("row_success_flag : ",row_success_flag)
					print("********************************* massage target pose success percentage ",row_success_flag_cnt)

		input("============ Press `Enter` to back to the start point ...")
		pneumatic.turn_off_main_pressure()
		pneumatic.panda.set_joint_velocity_scale(0.2)
		pneumatic.panda.set_joint_acceleration_scale(0.2)
		pneumatic.panda.set_line_velocity_scale(0.1)
		pneumatic.panda.move_to_start()
  
		input("============ Press `Enter` to back to the home joint ...")
		pneumatic.panda.set_joint_velocity_scale(0.4)
		pneumatic.panda.set_joint_acceleration_scale(0.2)
		pneumatic.panda.set_line_velocity_scale(0.4)
		pneumatic.panda.go_to_home_joint()
  
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == "__main__":
	main()

