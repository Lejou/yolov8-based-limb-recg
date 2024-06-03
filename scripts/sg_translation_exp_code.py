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

	home_point = [0.5529751898286808, -0.13591935879002315, -0.03080594549428671, 0.9980300334108954, -0.0508316751044764, -0.02920751896103075, 0.02233031593102823]

 
 #test
 # home_point = [0.553395425381741, -0.12462439599731921, 0.037719830072463, 0.9921745875837689, -0.12323204681714764, 0.009871963588530217, 0.017504470746581173]
	pneumatic.panda.set_start_point(home_point)
 
	wpose = pneumatic.panda.read_current_pose()

	target_waypoints_record=numpy.array([

	home_point
	])

	# target rotation eular angles referred to world frame, unit degree
	rotation_array=numpy.array([
	# Ry
		# [0,0,0],
		# [0,5,0],
		# [0,10,0]
	# none
		[0,0,0]
	# Rx
		# [0,0,0],
		# [5,0,0],
		# [10,0,0]
	# Rx & Ry
		# [0,0,0],
		# [5,5,0],
		# [5,10,0],
		# [10,5,0],
		# [10,10,0]
	# Rxy Test
		# [0,0,0],
		# [0,5,0],
		# [0,10,0],
		# [5,0,0],
		# [10,0,0],
		# [5,5,0],
		# [5,10,0],
		# [10,5,0],
		# [10,10,0]

	])
 
	# print("target_waypoints_record ", target_waypoints_record)

	(row,col) = target_waypoints_record.shape
	(rotation_row,col) = rotation_array.shape
	wpose = geometry_msgs.msg.Pose()
	# wpose = pneumatic.panda.move_group.get_current_pose().pose
	target_waypoints = []
	test_time_ =2

	for i in range(row):
		wpose.position.x = target_waypoints_record[i,0]
		wpose.position.y = target_waypoints_record[i,1]
		wpose.position.z = target_waypoints_record[i,2]
		wpose.orientation.x=target_waypoints_record[i,3]
		wpose.orientation.y=target_waypoints_record[i,4]
		wpose.orientation.z=target_waypoints_record[i,5]
		wpose.orientation.w=target_waypoints_record[i,6]
		target_waypoints.append(copy.deepcopy(wpose))

	row_success_flag_cnt = 0
 
	try:
		print("Make sure you use it with position joint trajetory!")
		input("============ Press `Enter` to move to the home joint ...")
		pneumatic.turn_off_main_pressure()
		pneumatic.panda.set_joint_velocity_scale(0.2)
		pneumatic.panda.set_joint_acceleration_scale(0.2)
		# pneumatic.panda.move_to_start()
		# pneumatic.panda.go_to_home_joint()



		# input("============ Press `Enter` to move to the start point and turn on cycle pressure ...")
		pneumatic.panda.move_to_start()
  
		
		pneumatic.panda.set_joint_velocity_scale(0.1)
		pneumatic.panda.set_joint_acceleration_scale(0.1)
		print("Record data!")
		pneumatic.turn_on_recording()

		mode = input("============ Press `Enter` to move from top to down...")
		# pneumatic.turn_on_cycle_pressure(225,test_time_)
		if mode != 'c':
			# pneumatic.panda.go_to_waypoints_without_loop_times(down_target_waypoints,True)
			pneumatic.panda.go_to_waypoints(target_waypoints,True,3)
			for rotation_row_i in range(0,rotation_row):
				restart_key = input("Re_suck the point?")
				if restart_key == 'r' and rotation_row_i>=1:
					rotation_row_i -= 1
     
				print("============ massage target pose rotation ",rotation_array[rotation_row_i])

				# new_pose = pneumatic.new_pose_after_eular_rotation(0,30,0,target_waypoints[row_i])
				# new_pose = pneumatic.new_pose_after_eular_array_rotation(rotation_array[rotation_row_i],target_waypoints[row_i])
				pose_from_array = copy.deepcopy(target_waypoints[0])
				new_pose = pneumatic.new_pose_from_array_rotation([0,0,0],rotation_array[rotation_row_i],pose_from_array)
				row_success_flag = pneumatic.excute_cycle_massage_with_force_control(new_pose, test_time_,0,0.06,5)

				if row_success_flag:
					row_success_flag_cnt = row_success_flag_cnt+1	
			print("********************************* massage target pose success percentage ",row_success_flag_cnt)



		# mode = input("============ Press `Enter` to move to point ...")
		# while mode != 'c':
			
		# 	for row_i in range(0,row):
		# 		rot_arr = pneumatic.generate_rotation_array_rxy(rot_rx,rot_ry,19)
				
		# 		wpts = pneumatic.generate_wpts_from_rotation_array(rot_arr,target_waypoints[row_i])
		# 		pneumatic.panda.go_to_waypoints(wpts,True,3)
		# 	mode = input("============ Press `c` to stop ...")

		input("============ Press `Enter` to back to the start point ...")
		pneumatic.panda.set_joint_velocity_scale(0.2)
		pneumatic.panda.set_joint_acceleration_scale(0.2)
		pneumatic.panda.set_line_velocity_scale(0.1)
		pneumatic.panda.move_to_start()
  
		# input("============ Press `Enter` to back to the home joint ...")
		# pneumatic.panda.set_joint_velocity_scale(0.4)
		# pneumatic.panda.set_joint_acceleration_scale(0.2)
		# pneumatic.panda.set_line_velocity_scale(0.4)
		# pneumatic.panda.go_to_home_joint()
  
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == "__main__":
	main()

