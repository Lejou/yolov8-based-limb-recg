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
[0.5557863288976371, 0.0710855551150147, 0.14649712460572267, 0.995019465526585, -0.09838203014571682, 0.009040276962116896, 0.013267463642673423] ,

[0.5524075949532231, 0.010330214312276698, 0.14647111280877975, -0.9935918972008186, 0.10838913672987274, -0.01261478611632204, 0.029473293913587433] ,

[0.5448148252428724, -0.04865223069332535, 0.14146279371343834, 0.9943778688260267, -0.10539132614934064, 0.009427054436091115, 0.00405871058360666] ,

[0.5415941146856703, -0.1176067935372384, 0.15069898277688004, 0.9919500769824112, -0.0827883249159862, -0.012385102341222394, 0.09501336435041378] ,

[0.5361765188106231, -0.18655605487645038, 0.16186632957041042, 0.996673824572423, -0.044501907101620004, -0.02878664509412638, 0.06190301988985626] ,

[0.5314282125265934, -0.23763294175259306, 0.16676699939665282, 0.9963795113328084, -0.03991279599340312, -0.022826983373694012, 0.07150868765887397] ,

[0.5260555586132679, -0.2573922762415661, 0.17288099329671222, 0.9850072542420386, -0.016802331223885015, 0.029610867063773565, 0.16912284415590223]
])

# 	target_waypoints_record=numpy.array([
# [0.550669389303097, 0.045311214430359105, 0.14653175381315542, -0.9961064413245462, 0.08437024254321554, -0.02171027617635597, 0.013487801586079673] ,

# # [0.5393970511565186, -0.05144609140187735, 0.1433878110520911, 0.9869058768260339, -0.15543307629301598, 0.043053055246832964, 0.0018840028630635103],

# # [0.5366887850903663, -0.10776897611013145, 0.14305173269200422, 0.9804918780398799, -0.1480359780347988, 0.028666194504621014, 0.12608796769553118] 


# ])
	position_compensate_array=numpy.array([
		[0,0,-0.005],
		[0,0,-0.005],
		[0,0,0.005]
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

	# head = input("============ Choose the head, general:2;bio:1...")
 
	# if head == '1':
		# print("bio_inspired head !")
		# # bio_inspired head
		# ready_depth = -0.04
		# NP_depth = 0.02
	# else:
	print("general head !")
	# # general head
	ready_depth = -0.03
	NP_depth = 0.03

 
	try:
		print("Start the massage!")
		input(
			"============ Press `Enter` to move to the start point ..."
		)

		pneumatic.turn_off_main_pressure()
		pneumatic.panda.set_joint_velocity_scale(0.2)
		pneumatic.panda.set_joint_acceleration_scale(0.2)
		pneumatic.panda.move_to_start()
		pneumatic.turn_on_cycle_pressure(150,interval_time_)
		# pneumatic.turn_on_main_pressure(255,150)
		pneumatic.panda.set_joint_velocity_scale(0.2)
		pneumatic.panda.set_joint_acceleration_scale(0.2)
		print("Record data!")
		pneumatic.turn_on_recording()
  

		stop_flag = input("============ Press `Enter` to massage ...")
		while not stop_flag=='c':
			for row_i in range(0,row):
				print("============ massage pose ",row_i+1)
				new_pose = pneumatic.new_pose_from_array_rotation(position_compensate_array[row_i],numpy.array([0,0,0]),target_waypoints[row_i])
				# new_pose = pneumatic.new_pose_after_eular_array_rotation(numpy.array([0,0,0]),target_waypoints[row_i])
				pneumatic.excute_cycle_massage(new_pose, interval_time_,ready_depth,NP_depth)
				# pneumatic.excute_cycle_massage_with_force(new_pose, interval_time_,ready_depth,36,20)
			stop_flag =  input("============ Press `Enter` to massage ...")
    

		# input(
		# 	"============ Press `Enter` to back to the start point ..."
		# )
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

