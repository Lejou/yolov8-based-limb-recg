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
	# pneumatic.panda.move_group.set_end_effector_link = "/panda_link8"

	home_point = [0.4672796196148121, -0.013881452471300217, 0.30772250634195436, 0.9977763587437242, -0.06385422713891682, -0.01587633825352971, 0.010626264266721286]

	pneumatic.panda.set_start_point(home_point)
 
	target_waypoints_record=numpy.array([

    [0.49247288591674265, -0.015086362283145267, 0.3951763976219965, 0.9630907547105197, -0.26672001922687405, -0.032901185997688376, 0.015301682740241665] ,

    [0.49175840969878426, -0.03487075364513222, 0.393324367161371, 0.9967300669861299, -0.07152307363947696, -0.037160973255230184, 0.005717129499230525] ,

    [0.5047402400361545, -0.02776312118775284, 0.3804209756568207, 0.9971229233604675, 0.0704990923409152, -0.02670185165611367, 0.00792242427021939] ,

    [0.5249059981200972, -0.05805212649463289, 0.3520710386900927, 0.9937709938307938, 0.10498802325903178, -0.036934292736878205, 0.005708310848569027] ,

    [0.5182027556680983, -0.12216025414336482, 0.32232958665480205, -0.9915343610943962, -0.10239154639824453, 0.036072359266186405, 0.071234590560169] ,

    [0.5777116224269361, -0.10385190546601047, 0.30111725161842323, -0.9810699468183851, -0.11318101938190973, 0.14771224812800432, 0.05359951543077207] ,

    [0.5795527997132152, -0.03920155673801916, 0.28577809390844494, -0.974908323407461, 0.04917976704366705, 0.2134632856741736, 0.03960476150124034] ,

    [0.3904585356970749, -0.02073353746846388, 0.27306699301807397, -0.9945033901263051, 0.06083543588450849, -0.07179197751690497, 0.045912620620482104] ,

    [0.4029900170323584, 0.07931460062022366, 0.31756386254074664, 0.985595676753328, -0.07453367185410302, 0.05610636528799324, 0.1410601626940234] ,

    [0.41586842900641874, 0.10735357531637651, 0.26898422751451617, 0.9769924589884058, -0.07634475053901764, 0.04391719307303288, 0.19423824107911736] 
	])


	# print("target_waypoints_record ", target_waypoints_record)

	(row,col) = target_waypoints_record.shape
	wpose = geometry_msgs.msg.Pose()
	# wpose = pneumatic.panda.move_group.get_current_pose().pose
	# target_waypoints = []

	# for i in range(row):
	# 	wpose.position.x = target_waypoints_record[i,0]
	# 	wpose.position.y = target_waypoints_record[i,1]
	# 	wpose.position.z = target_waypoints_record[i,2]
	# 	wpose.orientation.x=target_waypoints_record[i,3]
	# 	wpose.orientation.y=target_waypoints_record[i,4]
	# 	wpose.orientation.z=target_waypoints_record[i,5]
	# 	wpose.orientation.w=target_waypoints_record[i,6]
	# 	target_waypoints.append(copy.deepcopy(wpose))
 
	try:
		print("Make sure you use it with position joint trajetory!")
		input("============ Press `Enter` to start calibration...")
		pneumatic.turn_off_main_pressure()
		pneumatic.panda.set_joint_velocity_scale(0.2)
		pneumatic.panda.set_joint_acceleration_scale(0.2)

		pneumatic.panda.move_to_start()
  
		
		pneumatic.panda.set_joint_velocity_scale(0.1)
		pneumatic.panda.set_joint_acceleration_scale(0.1)

		mode = input("============ Press `Enter` to move to point ...")

		for i in range(0,row):
			if mode == 'c':
				break	
			target_waypoints = []

			wpose.position.x = target_waypoints_record[i,0]
			wpose.position.y = target_waypoints_record[i,1]
			wpose.position.z = target_waypoints_record[i,2]
			wpose.orientation.x=target_waypoints_record[i,3]
			wpose.orientation.y=target_waypoints_record[i,4]
			wpose.orientation.z=target_waypoints_record[i,5]
			wpose.orientation.w=target_waypoints_record[i,6]
			target_waypoints.append(copy.deepcopy(wpose))
			pneumatic.panda.go_to_waypoints(target_waypoints,True,3)
			mode = input("============ Press `c` to stop ...")

		input("============ Press `Enter` to back to the start point ...")

		pneumatic.panda.set_joint_velocity_scale(0.2)
		pneumatic.panda.set_joint_acceleration_scale(0.2)
		pneumatic.panda.set_line_velocity_scale(0.1)
		pneumatic.panda.move_to_start()
  

  
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == "__main__":
	main()

