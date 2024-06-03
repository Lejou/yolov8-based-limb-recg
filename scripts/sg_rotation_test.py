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

	# home_point = [0.5286889490021094, -0.1389050686968094, 0.3252420799100697, 0.9998611758779029, 0.012269542854380361, -0.011253558895930662, 0.0007079367156619242]
	# home_point = [0.3129279628089934, -0.0012283569694851843, 0.590698804104345, -0.9998983398629836, -0.007557572510111366, 0.0015960924121002913, 0.011982528882549246]

	# home_point = [0.3093168372395851, -0.0005289069338092461, 0.4964990920851179, -0.9999109443593682, -0.008583227721996815, 0.007995215638068242, 0.006372605752804993] 
	# home_point = [0.5542648080383868, -0.12213222774843874, -0.049401400864396414, -0.055576094162131565, -0.998360559325272, -0.01334883052651355, 0.0030769243537452193]
	# home_point = [0.552341130397982, -0.11719824841601052, -0.04632115674718923, 1, 0,0,0]
	home_point = [0.5836928668307876, -0.08160757650615574, 0.04926372909341627, 0.9961215430171626, -0.0873898951204701, -0.0028842258522225515, 0.009833921343269053]
	
	pneumatic.panda.set_start_point(home_point)
 
	wpose = pneumatic.panda.read_current_pose()

	target_waypoints_record=numpy.array([

	home_point
	])


	# target rotation eular angles referred to world frame, unit degree
	rot_rx = 10
	rot_ry = 10

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

	down_wpose.position.z = target_waypoints_record[0,2]+0.01
	down_target_waypoints.append(copy.deepcopy(down_wpose))
	down_wpose.position.z = target_waypoints_record[0,2]-0.005
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
  
		pneumatic.turn_on_cycle_pressure(225,interval_time_)
		pneumatic.panda.set_joint_velocity_scale(0.1)
		pneumatic.panda.set_joint_acceleration_scale(0.1)
		print("Record data!")
		pneumatic.turn_on_recording()

		# mode = input("============ Press `Enter` to move from top to down...")
		# if mode != 'c':
		# 	pneumatic.panda.go_to_waypoints_without_loop_times(down_target_waypoints,True)
			


		mode = input("============ Press `Enter` to move to point ...")
		while mode != 'c':
			
			for row_i in range(0,row):
				rot_arr = pneumatic.generate_rotation_array_rxy(rot_rx,rot_ry,29)
				
				wpts = pneumatic.generate_wpts_from_rotation_array(rot_arr,target_waypoints[row_i])
				pneumatic.panda.go_to_waypoints(wpts,True,3)
			mode = input("============ Press `c` to stop ...")

		input("============ Press `Enter` to back to the start point ...")
		pneumatic.turn_off_NP_suction()
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

