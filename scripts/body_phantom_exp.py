#!/usr/bin/env python3

from __future__ import print_function
from asyncio import current_task

import copy
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
		[0.61851564982081, 0.19105040219543528, 0.1522516890987819, 0.99756705253491, -0.04526755197884404, -0.04807415010041995, 0.022351842179779754] ,
		# [0.5830984885398582, 0.1317988660709763, 0.15679635126151886, 0.9991610176407097, -0.034965536142133116, 0.010383751062839558, 0.01862431634682662] ,
		[0.5820984885398582, 0.1317988660709763, 0.15679635126151886, 0.9991610176407097, -0.034965536142133116, 0.010383751062839558, 0.01862431634682662] ,
		[0.5939999500104677, 0.058607690858965715, 0.15944914737950708, 0.9968161365904866, -0.06961465310476701, -0.038705963171538973, 0.0036606399129302442] ,
		# [0.5804622103370157, -0.011621623169972362, 0.1585255406675471, -0.9989941603016843, 0.03927735460944521, 0.01669624804625254, 0.013762265839488759] ,
		[0.5859579124672276, -0.011129769539320856, 0.15611611433646572, -0.9989941603016843, 0.03927735460944521, 0.01669624804625254, 0.013762265839488759] ,
		[0.5607566689529603, -0.06276540125671369, 0.14781560789155002, 0.9974964353652266, -0.06106146097911724, 0.008487169136440379, 0.03464834354018105] ,
		[0.5663130936806606, -0.10167769644249353, 0.16224744678711023, 0.990293775497538, -0.057145077817588194, -0.01284768681686687, 0.12604476962347252] ,
		[0.5637171332201382, -0.1504457214094456, 0.16894389171294077, 0.9948057680971893, -0.03903852994878835, -0.01774186126983112, 0.09232050127184625] ,
		[0.5630914125135628, -0.21535997018957354, 0.17706979167053888, 0.9967694385637501, -0.01639050891880974, -0.03322701390646341, 0.07126262165802108] ,
		[0.5354205093448164, -0.2667966491063536, 0.18452501989462367, 0.9883574242659225, 0.029943248032059798, 0.07155280680729116, 0.13089569648531835] ,
		[0.5596119918018727, -0.29653609421806254, 0.19760530329898524, 0.9856353872052965, -0.04124055421197671, 0.0009629440882490009, 0.16377012612419994]
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

	# row_i = 0

 
	try:
		print("Start the massage!")
		mode = input(
			"============ Press `Enter` to move to the start point ..."
		)

		pneumatic.turn_off_main_pressure()
		pneumatic.panda.set_joint_velocity_scale(0.2)
		pneumatic.panda.set_joint_acceleration_scale(0.2)
		pneumatic.panda.move_to_start()
		pneumatic.turn_on_cycle_pressure(225,interval_time_)
		pneumatic.panda.set_joint_velocity_scale(0.1)
		pneumatic.panda.set_joint_acceleration_scale(0.1)
		print("Record data!")
		pneumatic.turn_on_recording()

		if mode == '':
		
			input("============ Press `Enter` to massage ...")
			for row_i in range(0,row):
			# for row_i in range(3,6):
				print("============ massage pose ",row_i+1)
				pneumatic.excute_cycle_massage(target_waypoints[row_i], interval_time_,-0.03,0.02)
				# pneumatic.excute_cycle_massage(target_waypoints[row_i], interval_time_,-0.02,0.03)
		else:
			c = 'c'
			while 1:
				c = input("============ input step massage num ...")
				if c == '':	
					break
				num = ord(c)-ord('0')
				print("num :" ,num)
				if num < row+1:
					pneumatic.excute_cycle_massage(target_waypoints[num-1], interval_time_,-0.03,0.02)
					# pneumatic.excute_cycle_massage(target_waypoints[num-1], interval_time_,-0.02,0.04)

		input(
			"============ Press `Enter` to back to the start point ..."
		)
		pneumatic.turn_off_main_pressure()
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

