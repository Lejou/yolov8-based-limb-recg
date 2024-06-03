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

	home_point = [0.615501079460015, -0.010397619938384275, 0.37827482734617984, -0.9982221661723325, 0.058009563568676314, 0.0037989943578391233, 0.013145468706021879]

 
 #test
 # home_point = [0.553395425381741, -0.12462439599731921, 0.037719830072463, 0.9921745875837689, -0.12323204681714764, 0.009871963588530217, 0.017504470746581173]
	pneumatic.panda.set_start_point(home_point)
 
	wpose = pneumatic.panda.read_current_pose()

	target_waypoints_record=numpy.array([
     
# [0.6101395090322698, 0.24827539744977187, 0.1448789253787623, 0.9977914839433659, -0.01720396163641576, -0.031299216048540385, 0.05600539797414198] 

# [0.5907486747629062, -0.18124208328355482, 0.17650561768141898, 0.9799184465110232, -0.12884332119889175, -0.10124317581592966, 0.11360939573184978] 

[0.6101395090322698, 0.24827539744977187, 0.1448789253787623, 0.9977914839433659, -0.01720396163641576, -0.031299216048540385, 0.05600539797414198] ,

[0.6121996491947477, 0.1890482977645992, 0.15159046550522465, 0.994774076622083, -0.010165324113551805, -0.04488689758843785, 0.09114361215196289] ,

[0.6142820908710215, 0.11758744629750742, 0.15853196478196177, 0.9943634312229598, -0.036225443726408166, -0.04874635674423166, 0.08690220576553316] ,

[0.6162439721814484, 0.04733232588828911, 0.16261275505069356, 0.9953302153003846, -0.009180623514203835, -0.08326204137255107, 0.04796520758481063] ,

[0.6131884985219878, 0.004833966175979415, 0.1638098448703268, 0.9961440704718537, -0.007315350000515073, -0.07019577202342939, 0.05211336071191949] ,

[0.6071375089823067, -0.05009322316133287, 0.16689623302918355, 0.9948868793964919, -0.07821250056686656, -0.05415773236435224, 0.03389481241811263] ,

[0.6028447419734555, -0.09339552588033213, 0.16883240564773178, 0.99359171111552, -0.07716146478690208, -0.06850881886705247, 0.04613364960452603] ,

[0.6011406289868928, -0.13833082763458052, 0.17131505780023037, 0.9901547044072756, -0.09295993099684566, -0.08466371751096262, 0.06152052039830422] ,

[0.5907486747629062, -0.18124208328355482, 0.17650561768141898, 0.9799184465110232, -0.12884332119889175, -0.10124317581592966, 0.11360939573184978] ,

[0.5732414507739224, -0.22804647547854057, 0.19089202613841716, 0.9829952210194166, -0.0847326334882929, -0.07727798485223615, 0.14341699379486425] 

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
 
 
	waypoints = []
	# wpose = self.move_group.get_current_pose().pose

	# waypoints.append(copy.deepcopy(wpose))
	waypoints.append(copy.deepcopy(pneumatic.panda.start_pose))
 
 
 
 

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
		# pneumatic.panda.go_to_waypoints(waypoints,True,3)
		
		pneumatic.panda.set_joint_velocity_scale(0.1)
		pneumatic.panda.set_joint_acceleration_scale(0.1)
		print("Record data!")
		pneumatic.turn_on_recording()

		mode = ''
		# pneumatic.turn_on_cycle_pressure(225,test_time_)
  
		repeat_time= 0

		while repeat_time != 6:
			repeat_time +=1
			print("============ Massage times: ",repeat_time)
			input("============ Press `Enter` to start ...")

			for row_i in range(0,row):
				# mode = input("============ Press `Enter` to move from top to down...")
				# restart_key = input("Re_suck the point?")
				# if restart_key == 'r' and row_i>=1:
				# 	row_i -= 1
				print("============ Massage point: ",row_i+1)

				if mode != 'c':
					# pneumatic.panda.go_to_waypoints_without_loop_times(down_target_waypoints,True)
					# pneumatic.panda.go_to_waypoints(target_waypoints,True,3)
					for rotation_row_i in range(0,rotation_row):
						print("============ massage target pose rotation ",rotation_array[rotation_row_i])

						# new_pose = pneumatic.new_pose_after_eular_rotation(0,30,0,target_waypoints[row_i])
						# new_pose = pneumatic.new_pose_after_eular_array_rotation(rotation_array[rotation_row_i],target_waypoints[row_i])
						pose_from_array = copy.deepcopy(target_waypoints[row_i])
						new_pose = pneumatic.new_pose_from_array_rotation([0,0,0],rotation_array[rotation_row_i],pose_from_array)
						row_success_flag = pneumatic.excute_cycle_massage_with_force_control(new_pose, test_time_,-0.015,0.06,5)

						# restart_key = input("Re_suck the point?")
						# if restart_key == 'r' and rotation_row_i>=1:
						# 	rotation_row_i -= 1
		
						if row_success_flag:
							row_success_flag_cnt = row_success_flag_cnt+1	
					print("********************************* massage target pose success percentage ",row_success_flag_cnt)

					# mode = input("Re_suck the point?")



			# mode = input("============ Press `Enter` to move to point ...")
			# while mode != 'c':
				
			# 	for row_i in range(0,row):
			# 		rot_arr = pneumatic.generate_rotation_array_rxy(rot_rx,rot_ry,19)
					
			# 		wpts = pneumatic.generate_wpts_from_rotation_array(rot_arr,target_waypoints[row_i])
			# 		pneumatic.panda.go_to_waypoints(wpts,True,3)
			# 	mode = input("============ Press `c` to stop ...")

			# input("============ Press `Enter` to back to the start point ...")
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

