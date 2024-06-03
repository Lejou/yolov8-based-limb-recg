#!/usr/bin/env python3

from __future__ import print_function
# from asyncio import current_task

import copy
# from re import S

from time import sleep
from franka_msgs import msg

import rospy
from six.moves import input
from std_msgs.msg import String,Int16,Float32,Float32MultiArray,Bool
from control_msgs.msg import FollowJointTrajectoryActionFeedback,FollowJointTrajectoryActionResult
from geometry_msgs.msg import WrenchStamped,Quaternion
import tf.transformations 

import numpy

from geometry_msgs.msg import Quaternion, Pose, Point
from transforms3d.quaternions import quat2mat, mat2quat
import roslaunch
import threading 

import PandaMoveGroup


class ArduinoPneumaticActuator(object):
    
	def __init__(self):
		super(ArduinoPneumaticActuator, self).__init__()
		# super().__init__(**kwargs)

		self.panda = PandaMoveGroup.MoveGroupPanda()

		self.sg_ratio = numpy.array([-9.7544,-13.3309,-9.2694,-6.2721,-11.7284,-9.8689])
		# self.sg_ratio = numpy.array([-10,-10,-10,-10,-10 ,-10])
		
		self.main_pwm_ = 0
		self.main_pressure_ = 0
		self.lip_pressure_ = 0
		self.strain_gauge_val_ = []
		self.main_value_limit_factor_ = 0.25
		self.main_valve_limit_ = 15  # negative pressure with positive value
		self.main_valve_trigger_pressure_ = 20 #self.main_valve_limit_*self.main_value_limit_factor_
		self.trajectory_state_ = 0
		self.ready_waypoints_ = []
		self.grasp_waypoints_ = []
		self.external_force = []
		self.base_sg_value = numpy.arange(6)
		self.sealing_force = numpy.arange(6)
		self.interval_massage_state = False
		# self.ready_grasp_pose = self.panda.move_group.get_current_pose().pose
		# self.deep_grasp_pose = self.panda.move_group.get_current_pose().pose
		# self.quit_grasp_pose = self.panda.move_group.get_current_pose().pose

		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)
		self.launch_start_force = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/robot/a_panda_ws/src/franka_controllers/franka_effort_controller/launch/start_force_control.launch"])
		self.launch_stop_force = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/robot/a_panda_ws/src/franka_controllers/franka_effort_controller/launch/stop_force_control.launch"])

		self.mutex = threading.Lock()

		self.data_record_pub = rospy.Publisher("data_record_topic",Bool,queue_size=10)
		self.main_pwm_pub = rospy.Publisher("main_pressure_pwm",Int16,queue_size=10)
		self.main_mode_pub = rospy.Publisher("main_valve_switch",Int16,queue_size=10) # 0: off 1:succescive 2:internal
		self.main_interval_freq_pub = rospy.Publisher("main_interval_freq",Float32,queue_size=10)
		self.main_interval_duty_pub = rospy.Publisher("main_interval_duty_cycle",Float32,queue_size=10)
		self.main_valve_limit_pub = rospy.Publisher("main_valve_threshold",Int16,queue_size=10)
		self.main_pressure_value_sub = rospy.Subscriber("main_pressure_val",Float32,self.main_pressure_value_callback,queue_size=10)
		self.interval_massage_sub = rospy.Subscriber("interval_massage_state",Bool,self.interval_massage_state_callback,queue_size=10)
		# self.panda_trajectory_result_sub = rospy.Subscriber("/franka_joint_trajectory_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback,self.trajectory_state_callback,queue_size=10)
		self.panda_trajectory_result_sub = rospy.Subscriber("/franka_joint_trajectory_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult,self.trajectory_state_callback,queue_size=10)
		self.panda_car_imp__trajectory_result_sub = rospy.Subscriber("/car_imp_joint_trajectory_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult,self.car_imp_trajectory_state_callback,queue_size=10)
		# http://docs.ros.org/en/diamondback/api/control_msgs/html/msg/FollowJointTrajectoryActionResult.html 

		# 20230301 add lip_control and strain gauge
		self.lip_switch_pub = rospy.Publisher("lip_swtich",Int16,queue_size=10)  # low voltage active, 0: open; 255:close No velocity adjustment
		self.lip_limit_pressure_pub = rospy.Publisher("lip_pressure_sub",Float32,queue_size=10)  # Set lip pressure limit

		self.lip_pressure_value_sub = rospy.Subscriber("lip_pressure_val",Float32,self.lip_pressure_value_callback,queue_size=10) # Get lip pressure
		self.strain_gauge_sub = rospy.Subscriber("strain_gauge_values",Float32MultiArray,self.strain_gauge_callback,queue_size=10) # Get vertical force from strain gauge
		self.sg_sealing_force_pub = rospy.Publisher("sg_sealing_force",Float32MultiArray,queue_size=10)

		self.ext_force_sub = rospy.Subscriber("/franka_state_controller/F_ext",WrenchStamped,self.ext_force_callback,queue_size=10) # Get external force
		self.recovery_pub = rospy.Publisher("recovery_flag_topic",Bool,queue_size=10) # Get external force

	def recovery_flag_pub(self):
		msg = Bool()
		msg.data = True
		self.recovery_pub.publish(msg)
  
	def turn_on_recording(self):
		msg = Bool()
		msg.data = True
		self.data_record_pub.publish(msg)

	def turn_off_recording(self):
		msg = Bool()
		msg.data = False
		self.data_record_pub.publish(msg)

	def car_imp_trajectory_state_callback(self,data):
		
		self.trajectory_state_ = copy.deepcopy(data.status.status)
		# print("Receive self.car_imp_trajectory_state_callback ",self.trajectory_state_)	
		# uint8 PENDING=0
		# uint8 ACTIVE=1
		# uint8 PREEMPTED=2
		# uint8 SUCCEEDED=3
		# uint8 ABORTED=4
		# uint8 REJECTED=5
		# uint8 PREEMPTING=6
		# uint8 RECALLING=7
		# uint8 RECALLED=8
		# uint8 LOST=9
	def trajectory_state_callback(self,data):
		
		self.trajectory_state_ = copy.deepcopy(data.status.status)
		# print("Receive self.trajectory_state_ ",self.trajectory_state_)	
		# uint8 PENDING=0
		# uint8 ACTIVE=1
		# uint8 PREEMPTED=2
		# uint8 SUCCEEDED=3
		# uint8 ABORTED=4
		# uint8 REJECTED=5
		# uint8 PREEMPTING=6
		# uint8 RECALLING=7
		# uint8 RECALLED=8
		# uint8 LOST=9
	def get_trajectory_state(self):
		
		return self.trajectory_state_ 
  
	def main_pressure_value_callback(self,data):
		self.main_pressure_ = int(-data.data * 7.5) # unit: mm/Hg
		# self.main_pressure_ = math.ceil(-data.data * 7.5)
  
	def interval_massage_state_callback(self,data):
		self.interval_massage_state = data.data 

	def lip_pressure_value_callback(self,data):
		# print(data.data)
		self.lip_pressure_ = data.data
	def strain_gauge_callback(self,data):
		# self.strain_gauge_val_ = numpy.array([data.data[0],data.data[1],data.data[2],data.data[3],data.data[4],data.data[5]])
		self.strain_gauge_val_ = numpy.array(data.data)

	def ext_force_callback(self,data):
		self.external_force = [data.wrench.force.x,data.wrench.force.y,data.wrench.force.z,data.wrench.torque.x,data.wrench.torque.y,data.wrench.torque.z]

	def turn_on_main_pressure(self,main_pwm,main_valve_limit):# main_valve_limit : positive with unit mm/Hg
		self.main_pwm_ = int(main_pwm)
		# self.main_valve_limit_ = main_valve_limit
		# self.main_valve_trigger_pressure_ = main_valve_limit*self.main_value_limit_factor_
  
		pwm_msg = Int16()
		pwm_msg.data = int(main_pwm)
		self.main_pwm_pub.publish(pwm_msg)

		valve_msg = Int16()
		valve_msg.data = main_valve_limit #-math.ceil(main_valve_limit/7.5)
		self.main_valve_limit_pub.publish(valve_msg)

		mode_msg = Int16()
		mode_msg.data = 1 
		self.main_mode_pub.publish(mode_msg)
		sleep(0.02)

	def turn_on_cycle_pressure(self,main_valve_limit):# main_valve_limit : positive with unit mm/Hg
		# self.main_pwm_ = int(main_pwm)
		# self.main_valve_limit_ = main_valve_limit
		# self.main_valve_trigger_pressure_ = main_valve_limit*self.main_value_limit_factor_
  
		# pwm_msg = Int16()
		# pwm_msg.data = int(main_pwm)
		# self.main_pwm_pub.publish(pwm_msg)

		valve_msg = Int16()
		valve_msg.data = main_valve_limit #-math.ceil(main_valve_limit/7.5)
		self.main_valve_limit_pub.publish(valve_msg)

		mode_msg = Int16()
		mode_msg.data = 2
		self.main_mode_pub.publish(mode_msg)
		sleep(0.02)

	def turn_on_cycle_pressure(self,main_valve_limit,interval_time):# main_valve_limit : positive with unit mm/Hg
		# self.main_pwm_ = int(main_pwm)
		# self.main_valve_limit_ = main_valve_limit
		# self.main_valve_trigger_pressure_ = main_valve_limit*self.main_value_limit_factor_
  
		# pwm_msg = Int16()
		# pwm_msg.data = int(main_pwm)
		# self.main_pwm_pub.publish(pwm_msg)

		interval_time_ = Float32()
		interval_time_.data = interval_time #-math.ceil(main_valve_limit/7.5)
		self.main_interval_freq_pub.publish(interval_time_)

		valve_msg = Int16()
		valve_msg.data = main_valve_limit #-math.ceil(main_valve_limit/7.5)
		self.main_valve_limit_pub.publish(valve_msg)

		mode_msg = Int16()
		mode_msg.data = 2
		self.main_mode_pub.publish(mode_msg)
		sleep(0.02)

	def turn_off_main_pressure(self):
		mode_msg = Int16()
		mode_msg.data = 0
		self.main_mode_pub.publish(mode_msg)
  
		self.main_pwm_ = 0
  
		pwm_msg = Int16()
		pwm_msg.data = self.main_pwm_
		self.main_pwm_pub.publish(pwm_msg)
		sleep(0.05)

	def interval_time(self,interval_time_data):
		# print("Interval time ", interval_time_data)
		sleep(interval_time_data)
		
	def set_main_limit_pressure(self,data):
		main_limit_pressure = Float32()
		main_limit_pressure.data = data
		self.main_valve_limit_pub.publish(main_limit_pressure)
		sleep(0.02)
	def set_lip_limit_pressure(self,data):
		lip_limit_pressure = Float32()
		lip_limit_pressure.data = data
		self.lip_limit_pressure_pub.publish(lip_limit_pressure)
		sleep(0.02)
	def turn_on_lip(self):
		lip_witch = Int16()
		lip_witch.data = 0
		self.lip_switch_pub(lip_witch)
		sleep(0.02)
	def turn_on_lip_with_limit_pressure(self,data):
		self.set_lip_limit_pressure(data)
		lip_witch = Int16()
		lip_witch.data = 0
		self.lip_switch_pub.publish(lip_witch)
		sleep(0.02)
	def turn_off_lip(self):
		lip_witch = Int16()
		lip_witch.data = 255
		self.lip_switch_pub.publish(lip_witch)
		sleep(0.05)
	def get_base_sg_value(self):
		self.base_sg_value = copy.deepcopy(self.strain_gauge_val_)
		# print("base_sg_value: ",self.base_sg_value)

	def get_sealing_force(self):
		print("strain_gauge_val_: ",self.strain_gauge_val_)
		print("base_sg_value: ",self.base_sg_value)
		# print("sg_ratio: ",self.sg_ratio)
		delta_sg_value = (self.strain_gauge_val_ - self.base_sg_value)
		self.sealing_force =self.sg_ratio * delta_sg_value
		# print("delta_sg_value: ",delta_sg_value)
		print("sealing_force: ",self.sealing_force)
		msg = Float32MultiArray()
		msg.data = [self.sealing_force[0],self.sealing_force[1],self.sealing_force[2],self.sealing_force[3],self.sealing_force[4],self.sealing_force[5]]
		self.sg_sealing_force_pub.publish(msg)

	def get_main_pressure_value(self):
		# print("self.main_pressure_ ",self.main_pressure_)
		# print("self.main_valve_trigger_pressure_ ",self.main_valve_trigger_pressure_)
		return self.main_pressure_

	def get_lip_pressure_value(self):
		return self.lip_pressure_

	def get_strain_gauge_value(self):
		return self.strain_gauge_val_

	def massage_once(self,main_pwm,main_valve_limit,interval_time_):
		self.turn_on_main_pressure(main_pwm,main_valve_limit)
		self.interval_time(interval_time_)
		self.turn_off_main_pressure()

	def transform_matrix_to_ros_pose(self,mat):
		"""
		Convert a transform matrix to a ROS pose.
		"""
		quat = mat2quat(mat[:3, :3])
		msg = Pose()
		msg.position = Point(x=mat[0, 3], y=mat[1, 3], z=mat[2, 3])
		msg.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
		return msg


	def ros_pose_to_transform_matrix(self,msg):
		"""
		Convert a ROS pose to a transform matrix
		"""
		mat44 = numpy.eye(4)
		mat44[:3, :3] = quat2mat([msg.orientation.w, msg.orientation.x,
								msg.orientation.y, msg.orientation.z])
		
		mat44[0:3, -1] = [msg.position.x, msg.position.y, msg.position.z]
		return mat44

	def find_neighbor_pose(self,distance_z,pose):
		ready_grasp_pose = copy.deepcopy(pose)
		trans_matrix_cur_pose = self.ros_pose_to_transform_matrix(pose)
		trans_matrix_ready_pose = copy.deepcopy(trans_matrix_cur_pose)

		ready_grasp_distance =numpy.array([[0,0,distance_z,1]]).T 

		trans_matrix_ready_position = numpy.dot(trans_matrix_cur_pose,ready_grasp_distance)
		# print("trans_matrix_ready_position ", trans_matrix_ready_position)

		trans_matrix_ready_pose[0:3, 3] = trans_matrix_ready_position[0:3,0]
		# print("trans_matrix_ready_pose ", trans_matrix_ready_pose)

		ready_grasp_pose = self.transform_matrix_to_ros_pose(trans_matrix_ready_pose)
		# print("ready_grasp_pose ", ready_grasp_pose)
		return ready_grasp_pose


	def excute_massage(self,target_pose,interval_time_data):

		self.ready_waypoints_ = []
		self.trajectory_state_ = 1
		ready_grasp_pose1 = self.find_neighbor_pose(-0.03, target_pose)
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
		# print(self.ready_waypoints_)
		ready_pose_position_state = self.panda.go_to_waypoints(self.ready_waypoints_,True,3)

		ready_grasp_pose2 = self.find_neighbor_pose(-0.01, target_pose)
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose2))

		if(not ready_pose_position_state):
			return False
		else:
			self.turn_on_main_pressure(250,250)

		deep_grasp_pose = self.find_neighbor_pose(0.02, target_pose)
		self.grasp_waypoints_ = []
		self.grasp_waypoints_.append(copy.deepcopy(deep_grasp_pose))
		# print(self.grasp_waypoints_)
		self.trajectory_state_ = 1
		suction_state = self.panda.go_to_waypoints(self.grasp_waypoints_,False,3)
		position_state = False
		pressure_state = False

		if(suction_state): # planning check
			print("self.trajectory_state_ ",self.trajectory_state_)	

			while(self.trajectory_state_ < 2): # trajectory running state
				
				if(self.get_main_pressure_value() >= self.main_valve_trigger_pressure_):
					print("!!!!!!!!!!!!Pressure Limit!")
					self.panda.move_group.stop()
					position_state = True
					pressure_state = True
					break
			if(self.trajectory_state_ == 3):
				position_state = True
				pressure_state = False
				print("Trajectory finished with no pressure!")
				# print("wpose",wpose)	
				# print("current_pose",current_pose)
			elif(self.trajectory_state_ == 4):
				print("Trajectory Aborted!")
				position_state = False
				pressure_state = False
				# return False
			elif(not self.trajectory_state_ == 1):
				print("Trajectory running problems. self.trajectory_state_ ",self.trajectory_state_)	
				# return False
				
			if(not pressure_state and position_state):
				print("!!!!!!!!!!!!Massage not working during trajectory!")
				pressure_cnt = 0
				if(self.get_main_pressure_value() < 20):
					pressure_state = False
					print("!!!!!!!!!!!!Low pressure. No hope for massage!")
					# return False

				else:
					pressure_meassure_flag = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
					while(not pressure_meassure_flag):
						if(pressure_cnt<50):
							self.interval_time(0.1)
							pressure_cnt =pressure_cnt+1
							pressure_state = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
							pressure_meassure_flag = pressure_state
						else:
							pressure_meassure_flag = True
							pressure_state = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
			if(pressure_state):
				print("!!!!!!!!!!!!Massage is working!")
				self.interval_time(interval_time_data)
				self.turn_off_main_pressure()
				return True
		else:
			print("!!!!!!!!!!!!trajectory planning failed!")
		self.turn_off_main_pressure()
		return False


	def excute_cycle_massage(self,target_pose,interval_time_data,ready_z,grasp_z):

		massage_flag = False

		self.ready_waypoints_ = []
		# self.trajectory_state_ = 0
		ready_grasp_pose1 = self.find_neighbor_pose(ready_z, target_pose)
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
		# print(self.ready_waypoints_)
		ready_pose_position_state = False

		# ready_grasp_pose2 = self.find_neighbor_pose(-0.01, target_pose)
		# self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose2))
		ready_pose_position_traj_cnt = 0
		while(not ready_pose_position_state):
			ready_pose_position_state = self.panda.go_to_waypoints(self.ready_waypoints_,True,3)
			ready_pose_position_traj_cnt = ready_pose_position_traj_cnt+1
			# print("ready_pose_position_traj_cnt", ready_pose_position_traj_cnt)
			# print("ready_pose_position_state", ready_pose_position_state)
			if(ready_pose_position_traj_cnt == 2):
				break
		if(not ready_pose_position_state):
			print("!!!!!!!!!!!!Unavailable ready point!", ready_pose_position_state)
			# print("self.trajectory_state_ ",self.trajectory_state_)	
			return massage_flag
		# else:
		# 	self.turn_on_cycle_pressure(250,interval_time_data)

		self.recovery_flag_pub()
  
		self.grasp_waypoints_ = []
		deep_grasp_pose = self.find_neighbor_pose(grasp_z, target_pose)
		self.grasp_waypoints_.append(copy.deepcopy(deep_grasp_pose))
		# print(self.grasp_waypoints_)
		self.trajectory_state_ = 1  # must re-initialize the flag to '1' as active status
		
		sleep(1)
		suction_state = False
		grasp_pose_position_traj_cnt = 0
		while(not suction_state):
			suction_state = self.panda.go_to_waypoints(self.grasp_waypoints_,False,3)
			grasp_pose_position_traj_cnt = grasp_pose_position_traj_cnt+1
			# print("ready_pose_position_traj_cnt", ready_pose_position_traj_cnt)
			# print("grasp_waypoints_ suction_state", suction_state)
			if(grasp_pose_position_traj_cnt == 2):
				break
		if(not suction_state):
			print("!!!!!!!!!!!!Unavailable suction_state!", grasp_pose_position_traj_cnt)
			# print("self.trajectory_state_ ",self.trajectory_state_)	
			return massage_flag
  
  
		# suction_state = self.panda.go_to_waypoints(self.grasp_waypoints_,False,3)
		# sleep(2)
		position_state = False
		pressure_state = False

		if(suction_state): # planning check
			print("self.trajectory_state_ ",self.get_trajectory_state())	

			while(self.get_trajectory_state() < 2): # trajectory running state
				
				if(self.get_main_pressure_value() >= self.main_valve_trigger_pressure_):
					print("1 !!!!!!!!!!!!Trajectory working on! get_main_pressure_value ",self.get_main_pressure_value())
					# sleep(0.5)
					self.panda.move_group.stop()
					position_state = True
					pressure_state = True
					massage_flag = True
					break
   
			print("self.trajectory_state_ ",self.get_trajectory_state())	
			if(self.get_trajectory_state() == 3):
				position_state = True
				# now1 = rospy.get_rostime()
				# sleep(5)
				# now2 = rospy.get_rostime()
    
				pressure_cnt_ = 0
				while(not self.get_main_pressure_value() >= self.main_valve_trigger_pressure_):
					pressure_cnt_ += 1
					# if pressure_cnt_%10 ==0:
					# 	print("3 pressure_cnt_:",pressure_cnt_)
					self.interval_time(0.1)
					if(pressure_cnt_ > 50):
						print("3 !!!!!!!!!!!!Massage is time out when position is right!")
						break
  
				print("3 get_main_pressure_value ",self.get_main_pressure_value())
				# print("sleep time: ",now2.secs -now1.secs)
				if(self.get_main_pressure_value() >= self.main_valve_trigger_pressure_):
					# print("3 !!!!!!Pressure working on!",self.get_trajectory_state())
					pressure_state = True
					massage_flag = True
				else:
					print("3 Trajectory finished with no pressure!",self.get_trajectory_state())
					pressure_state = False
					massage_flag = False
					
				# print("wpose",wpose)	
				# print("current_pose",current_pose)
			elif(self.get_trajectory_state() == 4):
				# print("Trajectory Aborted!",self.trajectory_state_)
				position_state = False
				# sleep(5)
				pressure_cnt_ = 0
				while(not self.get_main_pressure_value() >= self.main_valve_trigger_pressure_):
					pressure_cnt_ += 1
					if pressure_cnt_%10 ==0:
						print("4 pressure_cnt_:",pressure_cnt_)
					self.interval_time(0.1)
					if(pressure_cnt_ > 50):
						print("4 !!!!!!!!!!!!Massage is time out when Trajectory Aborted!")
						break
  
				# print("4 get_main_pressure_value ",self.get_main_pressure_value())
				if(self.get_main_pressure_value() >= self.main_valve_trigger_pressure_):
					print("4 Trajectory Aborted!!",self.get_trajectory_state())
					massage_flag = True
					pressure_state = True
				else:
					print("4 Trajectory Aborted!!!Failed",self.get_trajectory_state())
					massage_flag = False
					pressure_state = False
				# return False
			elif(not self.get_trajectory_state() == 1):
				print("Trajectory running problems. self.trajectory_state_: ",self.get_trajectory_state())	
				# return False

			# if(not pressure_state and position_state):
			# 	print("!!!!!!!!!!!!Massage not working during trajectory!")
			# 	pressure_cnt = 0
			# 	if(self.get_main_pressure_value() < self.main_valve_trigger_pressure_):
			# 		pressure_state = False
			# 		massage_flag = False
					
			# 		print("!!!!!!!!!!!!Low pressure. No hope for massage!")
			# 		# return False

			# 	else:
					# pressure_meassure_flag = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
					# while(not pressure_meassure_flag):
					# 	if(pressure_cnt<50):
					# 		self.interval_time(0.1)
					# 		pressure_cnt =pressure_cnt+1
					# 		pressure_state = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
					# 		pressure_meassure_flag = pressure_state
					# 	else:
					# 		pressure_meassure_flag = True
					# 		pressure_state = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
			if(pressure_state):
				# print("!!!!!!!!!!!!Massage is working! self.interval_massage_state:",self.interval_massage_state)
				massage_flag = True
				pressure_cnt_ = 0
				while(not self.interval_massage_state):
					pressure_cnt_ += 1
					# if pressure_cnt_%10 ==0:
					# 	print("pressure_cnt_:",pressure_cnt_)
					self.interval_time(0.1)
					if(pressure_cnt_ > interval_time_data*2*10):
						print("!!!!!!!!!!!!Massage is time out!")
						break
				# self.interval_time(2)
				# print("!self.interval_massage_state:",self.interval_massage_state)	
				# self.interval_time(interval_time_data)
				# self.turn_off_main_pressure()
				# return True
		else:
			print("!!!!!!!!!!!!trajectory planning failed!")
   
		print("self.trajectory_state_ ",self.get_trajectory_state())	
  
  
		# input("============ Press `Enter` to back to the ready point ...")
		# self.panda.read_current_pose_with_eular()
		self.ready_waypoints_ = []
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
		# print("~~Back to massage ready point!")
		# self.trajectory_state_ = 1
		ready_pose_position_state = self.panda.go_to_waypoints(self.ready_waypoints_,True,3)

		return massage_flag

	def rotate_from_eular(self,deg_rx,deg_ry,deg_rz,q_orig):
		q_rot = Quaternion()
		DE2RA = 3.1415926 / 180

		# RPY转四元素
		q_rot = tf.transformations.quaternion_from_euler(deg_rx * DE2RA, deg_ry * DE2RA, deg_rz * DE2RA)

		# print("q_rot: ",q_rot)

		# q_orig = quaternion_from_euler(0, 0, 0)
		# # Rotate the previous pose by 180* about X
		# q_rot = quaternion_from_euler(3.14159, 0, 0)
		# q_new = quaternion_multiply(q_rot, q_orig)
		return tf.transformations.quaternion_multiply(q_rot, q_orig)


	def rotate_from_eular_array(self,deg_rxyz,q_orig):
		q_rot = Quaternion()
		DE2RA = 3.1415926 / 180

		# RPY转四元素
		q_rot = tf.transformations.quaternion_from_euler(deg_rxyz[0] * DE2RA, deg_rxyz[1] * DE2RA, deg_rxyz[2] * DE2RA)

		# print("q_rot: ",q_rot)

		# q_orig = quaternion_from_euler(0, 0, 0)
		# # Rotate the previous pose by 180* about X
		# q_rot = quaternion_from_euler(3.14159, 0, 0)
		# q_new = quaternion_multiply(q_rot, q_orig)
		return tf.transformations.quaternion_multiply(q_rot, q_orig)



	def new_pose_after_eular_rotation(self,deg_rx,deg_ry,deg_rz,pose_orig): # rotate from the world frame
		q_orig = [pose_orig.orientation.x,pose_orig.orientation.y,pose_orig.orientation.z,pose_orig.orientation.w]
		q_new = self.rotate_from_eular(deg_rx,deg_ry,deg_rz,q_orig)
		pose_new = Pose()
		pose_new.position = copy.deepcopy(pose_orig.position)

		pose_new.orientation.x=q_new[0]
		pose_new.orientation.y=q_new[1]
		pose_new.orientation.z=q_new[2]
		pose_new.orientation.w=q_new[3]
		
		return pose_new
	
	def new_pose_after_eular_array_rotation(self,deg_rxyz,pose_orig): # rotate from the world frame
		q_orig = [pose_orig.orientation.x,pose_orig.orientation.y,pose_orig.orientation.z,pose_orig.orientation.w]
		q_new = self.rotate_from_eular_array(deg_rxyz,q_orig)
		pose_new = Pose()
		pose_new.position = copy.deepcopy(pose_orig.position)

		pose_new.orientation.x=q_new[0]
		pose_new.orientation.y=q_new[1]
		pose_new.orientation.z=q_new[2]
		pose_new.orientation.w=q_new[3]
		
		return pose_new

	def new_pose_from_array_rotation(self,compensate_position_array,deg_rxyz,pose_orig): # rotate from the world frame
		new_pose = self.new_pose_after_eular_array_rotation(deg_rxyz,pose_orig)

		new_pose.position.x = pose_orig.position.x + compensate_position_array[0]
		new_pose.position.y = pose_orig.position.y + compensate_position_array[1]
		new_pose.position.z = pose_orig.position.z + compensate_position_array[2]

		return new_pose

def main():
	pneumatic = ArduinoPneumaticActuator()
 
	wpose = pneumatic.panda.read_current_pose()
	# print("current_pose",wpose)
	pneumatic.panda.set_joint_velocity_scale(0.2)
	pneumatic.panda.set_joint_acceleration_scale(0.1)

	try:
		print("Press Ctrl-D to exit at any time")
		input(
			"============ 1 Press `Enter` to move to the start point ..."
		)
		pneumatic.panda.move_to_start()

		while " " != input("============ 2 Press `Enter` to repeat a trajectory ..."):
			waypoints = []
			wpose = pneumatic.panda.move_group.get_current_pose().pose
			wpose.position.x = 0.45
			wpose.position.y = 0
			wpose.position.z = 0.4
			waypoints.append(copy.deepcopy(wpose))
			
			wpose.position.y = 0.2
			waypoints.append(copy.deepcopy(wpose))

			wpose.position.y = -0.201
			waypoints.append(copy.deepcopy(wpose))
			
			# wpose.position.y = 0.2
			# waypoints.append(copy.deepcopy(wpose))

			# wpose.position.y = -0.201
			# waypoints.append(copy.deepcopy(wpose))
			
			# wpose.position.y = 0
			# wpose.position.x = 0.3501
			# waypoints.append(copy.deepcopy(wpose))

			# wpose.position.x = 0.5501
			# waypoints.append(copy.deepcopy(wpose))

			# wpose.position.x = 0.35
			# waypoints.append(copy.deepcopy(wpose))

			# wpose.position.x = 0.55
			# waypoints.append(copy.deepcopy(wpose))

			# wpose.position.x = 0.45
			# waypoints.append(copy.deepcopy(wpose))

			pneumatic.panda.go_to_waypoints(waypoints,True,3)

		input(
			"============ 3 Press `Enter` to move to the start point ..."
		)
		pneumatic.panda.move_to_start()
		print("============ Python tutorial demo complete!")
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == "__main__":
	main()

