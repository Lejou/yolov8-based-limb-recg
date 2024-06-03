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
	#rx
	# home_point = [0.5536832819277401, -0.1278601969340713, -0.020440385690624595, -0.9898954512337091, 0.05725454085092933, -0.019121251177847933, 0.12831006422261323]
	#ry
	# home_point = [0.5501538625548341, -0.1271494291871578, -0.02696497044433134, 0.9945064237757165, -0.05272963081552206, -0.0892127811421569, 0.014755853128890984]
	#rz
	# home_point = [0.5586561939548382, -0.13463586839945146, -0.010236343246660177, 0.9825077072769813, -0.18569461803707554, -0.005238599655562597, 0.012981089155531882]
	
 
 #test
 # home_point = [0.553395425381741, -0.12462439599731921, 0.037719830072463, 0.9921745875837689, -0.12323204681714764, 0.009871963588530217, 0.017504470746581173]
	pneumatic.panda.set_start_point(home_point)
 
	wpose = pneumatic.panda.read_current_pose()

	target_waypoints_record=numpy.array([

	home_point
	])


	# print("target_waypoints_record ", target_waypoints_record)

	(row,col) = target_waypoints_record.shape
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

	down_wpose = wpose
	down_target_waypoints = []

	down_target_waypoints.append(copy.deepcopy(down_wpose))
	down_wpose.position.z = target_waypoints_record[0,2]-0.06
	# down_wpose.position.z = target_waypoints_record[0,2]-0.05
	down_target_waypoints.append(copy.deepcopy(down_wpose))

 
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
		pneumatic.turn_on_NP_suction(0)
		if mode != 'c':
			# pneumatic.panda.go_to_waypoints_without_loop_times(down_target_waypoints,True)
			pneumatic.panda.go_to_waypoints(down_target_waypoints,True,3)
			


		# mode = input("============ Press `Enter` to move to point ...")
		# while mode != 'c':
			
		# 	for row_i in range(0,row):
		# 		rot_arr = pneumatic.generate_rotation_array_rxy(rot_rx,rot_ry,19)
				
		# 		wpts = pneumatic.generate_wpts_from_rotation_array(rot_arr,target_waypoints[row_i])
		# 		pneumatic.panda.go_to_waypoints(wpts,True,3)
		# 	mode = input("============ Press `c` to stop ...")

		input("============ Press `Enter` to back to the start point ...")
		pneumatic.turn_off_NP_suction()
		# sleep(2)
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

