#!/usr/bin/env python3

from __future__ import print_function
# from asyncio import current_task

import copy
import math
# from re import S

from time import sleep
from franka_msgs import msg
from genpy import Time

import rospy
from six.moves import input
from std_msgs.msg import String,Int16,Float32,Float32MultiArray,Bool
from control_msgs.msg import FollowJointTrajectoryActionFeedback,FollowJointTrajectoryActionResult
from geometry_msgs.msg import Quaternion, Pose, Point, Wrench,WrenchStamped,PoseArray
import tf

import numpy

from transforms3d.quaternions import quat2mat, mat2quat
import roslaunch
import threading 

import PandaMoveGroup
from limb_recg.srv import *


class ArduinoPneumaticActuator(object):
    
	def __init__(self):
		super(ArduinoPneumaticActuator, self).__init__()
		# super().__init__(**kwargs)

		self.panda = PandaMoveGroup.MoveGroupPanda()

		self.sg_ratio = numpy.array([-9.7544,-13.3309,-9.2694,-6.2721,-11.7284,-9.8689])
		# self.sg_ratio = numpy.array([-10,-10,-10,-10,-10 ,-10])
		
		self.main_pwm_ = 0
		self.main_pressure_ = 0 # represent negative pressure with positive value in mmHg
		self.lip_pressure_ = 0
		self.strain_gauge_val_ = []
		self.main_value_limit_factor_ = 0.25
		self.main_valve_limit_ = 75  # negative pressure with positive value unit mmHg
		self.main_valve_trigger_pressure_ = 5*7.5 #self.main_valve_limit_*self.main_value_limit_factor_
		self.trajectory_state_ = 0
		self.ready_waypoints_ = []
		self.grasp_waypoints_ = []
		self.external_force = []
		self.base_sg_value = numpy.arange(6)
		self.sealing_force = numpy.arange(6)
		self.interval_massage_state = False

		self.main_pressure_vel_start_time = rospy.get_rostime()
		self.main_pressure_vel_end_time = rospy.get_rostime()
		self.main_pressure_vel = Float32()
  
		self.translation_stiffness_ = 1200
		self.compensate_wrench_ = Wrench()

		self.suction_traj_ = None
		self.suction_mode_ = 0 # 1: cyclic; 2: continous
		self.suction_cyclic_time_=2
		self.suction_cyclic_ready_depth_= -0.01 # dis from the suction pose
		self.suction_cyclic_grasp_depth_ = 0.06
		self.suction_desired_force_ = 5

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
		self.main_valve_limit_pub = rospy.Publisher("main_valve_threshold",Float32,queue_size=10)
		self.main_pressure_value_sub = rospy.Subscriber("main_pressure_val",Float32,self.mainPressureValueCallback,queue_size=10)
		self.interval_massage_sub = rospy.Subscriber("interval_massage_state",Bool,self.intervalMassageStateCallback,queue_size=10)
		# self.panda_trajectory_result_sub = rospy.Subscriber("/franka_joint_trajectory_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback,self.trajectoryStateCallback,queue_size=10)
		panda_trajectory_result_sub = rospy.Subscriber("/franka_joint_trajectory_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult,self.trajectoryStateCallback,queue_size=10)
		panda_car_imp_trajectory_result_sub = rospy.Subscriber("/car_imp_joint_trajectory_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult,self.carImpTrajectoryStateCallback,queue_size=10)
		panda_NP_trajectory_result_sub = rospy.Subscriber("/franka_NP_joint_trajectory_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult,self.frankaNPTrajectoryStateCallback,queue_size=10)
		# http://docs.ros.org/en/diamondback/api/control_msgs/html/msg/FollowJointTrajectoryActionResult.html 

		# 20230301 add lip_control and strain gauge
		self.lip_switch_pub = rospy.Publisher("lip_swtich",Int16,queue_size=10)  # low voltage active, 0: open; 255:close No velocity adjustment
		self.lip_limit_pressure_pub = rospy.Publisher("lip_pressure_sub",Float32,queue_size=10)  # Set lip pressure limit

		self.lip_pressure_value_sub = rospy.Subscriber("lip_pressure_val",Float32,self.lipPressureValueCallback,queue_size=10) # Get lip pressure
		self.strain_gauge_sub = rospy.Subscriber("strain_gauge_values",Float32MultiArray,self.strainGaugeCallback,queue_size=10) # Get vertical force from strain gauge
		self.sg_sealing_force_pub = rospy.Publisher("sg_sealing_force",Float32MultiArray,queue_size=10)

		self.ext_force_sub = rospy.Subscriber("/franka_state_controller/F_ext",WrenchStamped,self.extForceCallback,queue_size=10) # Get external force
		self.recovery_pub = rospy.Publisher("recovery_flag_topic",Bool,queue_size=10) # Get external force
		
		self.main_pressure_vel_pub = rospy.Publisher("main_pressure_vel",Float32,queue_size=10) # Get external force
		self.compensate_wrench_pub = rospy.Publisher("compensate_wrench_topic",Wrench,queue_size=10) # Get external force
		self.robot_desired_force_pub = rospy.Publisher("/suction_desired_force",Float32,queue_size=10) # Get external force

		# rospy.Subscriber("/limb_suction_traj", PoseArray,self.limbSuctionTrajCB,queue_size=10)
		self.make_suction_srv_ = rospy.Service('make_suctions', SuctionPose, self.makeSuction)
		# rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)



	def limbSuctionTrajCB(self,data):
		self.suction_traj_ = data.poses

	def setTranslationStiffness(self,data):
		self.translation_stiffness_ = data
  
	def mainPressureVelTopicPub(self):
		self.main_pressure_vel_pub.publish(self.main_pressure_vel)

	def recoveryFlagPub(self):
		msg = Bool()
		msg.data = True
		self.recovery_pub.publish(msg)
  
	def turnOnRecording(self):
		msg = Bool()
		msg.data = True
		self.data_record_pub.publish(msg)

	def turnOffRecording(self):
		msg = Bool()
		msg.data = False
		self.data_record_pub.publish(msg)
  
	def frankaNPTrajectoryStateCallback(self,data):
		self.trajectory_state_ = copy.deepcopy(data.status.status)
  
	def carImpTrajectoryStateCallback(self,data):
		
		self.trajectory_state_ = copy.deepcopy(data.status.status)
		# print("Receive self.carImpTrajectoryStateCallback ",self.trajectory_state_)	
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
	def trajectoryStateCallback(self,data):
		
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
	def getTrajectoryState(self):
		
		return self.trajectory_state_ 
  
	def mainPressureValueCallback(self,data):
		# self.main_pressure_vel_start_time = copy.deepcopy(self.main_pressure_vel_end_time)
		# self.main_pressure_vel_end_time = rospy.get_rostime()
		# delta_time = self.main_pressure_vel_end_time.to_sec() - self.main_pressure_vel_start_time.to_sec()
		# self.main_pressure_vel.data =  (data.data -(-self.main_pressure_/7.5)) / delta_time
  
		self.main_pressure_ = int(-data.data * 7.5) # unit: mm/Hg
		# self.main_pressure_ = math.ceil(-data.data * 7.5)
  
	def intervalMassageStateCallback(self,data):
		self.interval_massage_state = data.data 

	def lipPressureValueCallback(self,data):
		# print(data.data)
		self.lip_pressure_ = data.data
	def strainGaugeCallback(self,data):
		# self.strain_gauge_val_ = numpy.array([data.data[0],data.data[1],data.data[2],data.data[3],data.data[4],data.data[5]])
		self.strain_gauge_val_ = numpy.array(data.data)

	def extForceCallback(self,data):
		self.external_force = [data.wrench.force.x,data.wrench.force.y,data.wrench.force.z,data.wrench.torque.x,data.wrench.torque.y,data.wrench.torque.z]

	def turnOnNPSuction(self,main_valve_limit):# main_valve_limit : positive with unit mm/Hg
		mode_msg = Int16()
		mode_msg.data = 1 
		self.main_mode_pub.publish(mode_msg)
		sleep(0.2)
		valve_msg = Float32()
		valve_msg.data = main_valve_limit #-math.ceil(main_valve_limit/7.5)
		self.main_valve_limit_ = main_valve_limit
		self.main_valve_limit_pub.publish(valve_msg)

	def turnOffNPSuction(self):# main_valve_limit : positive with unit mm/Hg
		mode_msg = Int16()
		mode_msg.data = 0
		self.main_mode_pub.publish(mode_msg)
		sleep(2)

	def turnOnMainPressure(self,main_pwm,main_valve_limit):# main_valve_limit : positive with unit mm/Hg
		self.main_pwm_ = int(main_pwm)
		# self.main_valve_limit_ = main_valve_limit
		# self.main_valve_trigger_pressure_ = main_valve_limit*self.main_value_limit_factor_
  
		pwm_msg = Int16()
		pwm_msg.data = int(main_pwm)
		self.main_pwm_pub.publish(pwm_msg)

		valve_msg = Float32()
		valve_msg.data = main_valve_limit #-math.ceil(main_valve_limit/7.5)
		self.main_valve_limit_ = main_valve_limit
		self.main_valve_limit_pub.publish(valve_msg)

		mode_msg = Int16()
		mode_msg.data = 1 
		self.main_mode_pub.publish(mode_msg)
		sleep(0.02)

	def turnOnCyclePressure(self,main_valve_limit):# main_valve_limit : positive with unit mm/Hg
		# self.main_pwm_ = int(main_pwm)
		# self.main_valve_limit_ = main_valve_limit
		# self.main_valve_trigger_pressure_ = main_valve_limit*self.main_value_limit_factor_
  
		# pwm_msg = Int16()
		# pwm_msg.data = int(main_pwm)
		# self.main_pwm_pub.publish(pwm_msg)

		valve_msg = Float32()
		valve_msg.data = main_valve_limit #-math.ceil(main_valve_limit/7.5)
		self.main_valve_limit_ = main_valve_limit

		self.main_valve_limit_pub.publish(valve_msg)

		mode_msg = Int16()
		mode_msg.data = 2
		self.main_mode_pub.publish(mode_msg)
		sleep(0.02)

	def turnOnCyclePressure(self,main_valve_limit,intervalTime):# main_valve_limit : positive with unit mm/Hg
		# self.main_pwm_ = int(main_pwm)
		# self.main_valve_limit_ = main_valve_limit
		# self.main_valve_trigger_pressure_ = main_valve_limit*self.main_value_limit_factor_
  
		# pwm_msg = Int16()
		# pwm_msg.data = int(main_pwm)
		# self.main_pwm_pub.publish(pwm_msg)

		interval_time_ = Float32()
		interval_time_.data = intervalTime #-math.ceil(main_valve_limit/7.5)
		self.main_interval_freq_pub.publish(interval_time_)

		valve_msg = Float32()
		valve_msg.data = main_valve_limit #-math.ceil(main_valve_limit/7.5)
		self.main_valve_limit_ = main_valve_limit

		self.main_valve_limit_pub.publish(valve_msg)

		mode_msg = Int16()
		mode_msg.data = 2
		self.main_mode_pub.publish(mode_msg)
		sleep(0.02)

	def turnOffMainPressure(self):
		valve_msg = Float32()
		valve_msg.data = 0 #-math.ceil(main_valve_limit/7.5)
		self.main_valve_limit_pub.publish(valve_msg)

		sleep(2)
		mode_msg = Int16()
		mode_msg.data = 0
		self.main_mode_pub.publish(mode_msg)
  
		# self.main_pwm_ = 0
		# pwm_msg = Int16()
		# pwm_msg.data = self.main_pwm_
		# self.main_pwm_pub.publish(pwm_msg)
		# sleep(0.05)

	def intervalTime(self,interval_time_data):
		# print("Interval time ", interval_time_data)
		sleep(interval_time_data)
		
	def setMainLimitPressure(self,data):
		main_limit_pressure = Float32()
		main_limit_pressure.data = data
		self.main_valve_limit_pub.publish(main_limit_pressure)
		sleep(0.02)
	def setLipLimitPressure(self,data):
		lip_limit_pressure = Float32()
		lip_limit_pressure.data = data
		self.lip_limit_pressure_pub.publish(lip_limit_pressure)
		sleep(0.02)
	def turnOnLip(self):
		lip_witch = Int16()
		lip_witch.data = 0
		self.lip_switch_pub(lip_witch)
		sleep(0.02)
	def turnOnLipWithLimitPressure(self,data):
		self.setLipLimitPressure(data)
		lip_witch = Int16()
		lip_witch.data = 0
		self.lip_switch_pub.publish(lip_witch)
		sleep(0.02)
	def turnOffLip(self):
		lip_witch = Int16()
		lip_witch.data = 255
		self.lip_switch_pub.publish(lip_witch)
		sleep(0.05)
	def getBaseSgValue(self):
		self.base_sg_value = copy.deepcopy(self.strain_gauge_val_)
		# print("base_sg_value: ",self.base_sg_value)

	def getSealingForce(self):
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

	def getMainPressureValue(self):
		# print("self.main_pressure_ ",self.main_pressure_)
		# print("self.main_valve_trigger_pressure_ ",self.main_valve_trigger_pressure_)
		return self.main_pressure_

	def getLipPressureValue(self):
		return self.lip_pressure_

	def getStrainGaugeValue(self):
		return self.strain_gauge_val_

	def massageOnce(self,main_pwm,main_valve_limit,interval_time_):
		self.turnOnMainPressure(main_pwm,main_valve_limit)
		self.intervalTime(interval_time_)
		self.turnOffMainPressure()

	def transformMatrixToRosPose(self,mat):
		"""
		Convert a transform matrix to a ROS pose.
		"""
		quat = mat2quat(mat[:3, :3])
		msg = Pose()
		msg.position = Point(x=mat[0, 3], y=mat[1, 3], z=mat[2, 3])
		msg.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
		return msg


	def rosPoseToTransformMatrix(self,msg):
		"""
		Convert a ROS pose to a transform matrix
		"""
		mat44 = numpy.eye(4)
		mat44[:3, :3] = quat2mat([msg.orientation.w, msg.orientation.x,
								msg.orientation.y, msg.orientation.z])
		
		mat44[0:3, -1] = [msg.position.x, msg.position.y, msg.position.z]
		return mat44

	def findNeighborPose(self,distance_z,pose):
		ready_grasp_pose = copy.deepcopy(pose)
		trans_matrix_cur_pose = self.rosPoseToTransformMatrix(pose)
		trans_matrix_ready_pose = copy.deepcopy(trans_matrix_cur_pose)

		ready_grasp_distance =numpy.array([[0,0,distance_z,1]]).T 

		trans_matrix_ready_position = numpy.dot(trans_matrix_cur_pose,ready_grasp_distance)
		# print("trans_matrix_ready_position ", trans_matrix_ready_position)

		trans_matrix_ready_pose[0:3, 3] = trans_matrix_ready_position[0:3,0]
		# print("trans_matrix_ready_pose ", trans_matrix_ready_pose)

		ready_grasp_pose = self.transformMatrixToRosPose(trans_matrix_ready_pose)
		# print("ready_grasp_pose ", ready_grasp_pose)
		return ready_grasp_pose


	def excuteMassage(self,target_pose,interval_time_data):

		self.ready_waypoints_ = []
		self.trajectory_state_ = 1
		ready_grasp_pose1 = self.findNeighborPose(-0.03, target_pose)
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
		# print(self.ready_waypoints_)
		ready_pose_position_state = self.panda.go_to_waypoints(self.ready_waypoints_,True,3)

		ready_grasp_pose2 = self.findNeighborPose(-0.01, target_pose)
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose2))

		if(not ready_pose_position_state):
			return False
		else:
			self.turnOnMainPressure(250,250)

		deep_grasp_pose = self.findNeighborPose(0.02, target_pose)
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
				
				if(self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
					print("!!!!!!!!!!!!Pressure Limit!")
					self.panda.move_group.stop()
					position_state = True
					pressure_state = True
					break


			if(self.trajectory_state_ == 3):
				position_state = True
				pressure_state = False
				# 
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
				if(self.getMainPressureValue() < 20):
					pressure_state = False
					print("!!!!!!!!!!!!Low pressure. No hope for massage!")
					# return False

				else:
					pressure_meassure_flag = self.getMainPressureValue() >= self.main_valve_trigger_pressure_
					while(not pressure_meassure_flag):
						if(pressure_cnt<50):
							self.intervalTime(0.1)
							pressure_cnt =pressure_cnt+1
							pressure_state = self.getMainPressureValue() >= self.main_valve_trigger_pressure_
							pressure_meassure_flag = pressure_state
						else:
							pressure_meassure_flag = True
							pressure_state = self.getMainPressureValue() >= self.main_valve_trigger_pressure_
			if(pressure_state):
				print("!!!!!!!!!!!!Massage is working!")
				self.intervalTime(interval_time_data)
				self.turnOffMainPressure()
				return True
		else:
			print("!!!!!!!!!!!!trajectory planning failed!")
		self.turnOffMainPressure()
		return False

	def excuteCycleMassageWithForceControl(self,target_pose,interval_time_data,ready_z,grasp_z,desired_force):
		Fz_desire = Float32()
		Fz_desire.data = desired_force #-math.ceil(main_valve_limit/7.5)
		self.robot_desired_force_pub.publish(Fz_desire)
  
		massage_flag = False
		# self.turnOnNPSuction(225)
  
		self.ready_waypoints_ = []
		# self.trajectory_state_ = 0
		ready_grasp_pose1 = self.findNeighborPose(ready_z, target_pose)
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
		# print(self.ready_waypoints_)
		ready_pose_position_state = False

		# ready_grasp_pose2 = self.findNeighborPose(-0.01, target_pose)
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
		# 	self.turnOnCyclePressure(250,interval_time_data)

		self.recoveryFlagPub()

  
		self.grasp_waypoints_ = []
		deep_grasp_pose = self.findNeighborPose(grasp_z, target_pose)
		self.grasp_waypoints_.append(copy.deepcopy(deep_grasp_pose))
		# print(self.grasp_waypoints_)
		self.trajectory_state_ = 1  # must re-initialize the flag to '1' as active status
		
		self.turnOnNPSuction(225)

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

		if(suction_state): # planning check
			# print("self.trajectory_state_ ",self.getTrajectoryState())	

			while(self.getTrajectoryState() < 2): # trajectory running state
				
				if(self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
					print("1 !!!!!!!!!!!!Trajectory working on! getMainPressureValue ",self.getMainPressureValue())
					# sleep(0.5)
					# self.panda.move_group.stop()
					break
 

			# print("self.trajectory_state_ ",self.getTrajectoryState())	
			if(self.getTrajectoryState() == 3):
				# position_state = True
				# now1 = rospy.get_rostime()
				# sleep(5)
				# now2 = rospy.get_rostime()
    
				pressure_cnt_ = 0
				while(not self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
					pressure_cnt_ += 1
					# if pressure_cnt_%10 ==0:
					# 	print("3 pressure_cnt_:",pressure_cnt_)
					self.intervalTime(0.1)
					if(pressure_cnt_ > 50):
						print("3 !!!!!!!!!!!!Massage is time out when position is right!")
						break
  
				print("3 getMainPressureValue ",self.getMainPressureValue())
				# print("sleep time: ",now2.secs -now1.secs)
					
				# print("wpose",wpose)	
				# print("current_pose",current_pose)
			elif(self.getTrajectoryState() == 4):
				# print("Trajectory Aborted!",self.trajectory_state_)
				# position_state = False
				# sleep(5)
				pressure_cnt_ = 0
				while(not self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
					pressure_cnt_ += 1
					if pressure_cnt_%10 ==0:
						print("4 pressure_cnt_:",pressure_cnt_)
					self.intervalTime(0.1)
					if(pressure_cnt_ > 50):
						print("4 !!!!!!!!!!!!Massage is time out when Trajectory Aborted!")
						break
  
				# print("4 getMainPressureValue ",self.getMainPressureValue())

				# return False
			elif(not self.getTrajectoryState() == 1):
				print("Trajectory running problems. self.trajectory_state_: ",self.getTrajectoryState())	
				# return False
			
			# # pose correction
			# pressure_cnt_ = 0
			# while(self.getMainPressureValue() < self.main_valve_trigger_pressure_):
			# 	self.intervalTime(0.1)
			# 	pressure_cnt_ += 1
			# 	if(pressure_cnt_ > 50):
			# 		print("pose correction done!")
			# 		break

			if(self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
				# print("3 !!!!!!Pressure working on!",self.getTrajectoryState())
				massage_flag = True
			else:
				print("5 Trajectory finished with no pressure!",self.getTrajectoryState())
				massage_flag = False
    
    
			if(massage_flag):
				# print("!!!!!!!!!!!!Massage is working! self.interval_massage_state:",self.interval_massage_state)
				pressure_cnt_ = 0
				while(not self.interval_massage_state):
					pressure_cnt_ += 1
					# if pressure_cnt_%10 ==0:
					# 	print("pressure_cnt_:",pressure_cnt_)
					self.intervalTime(0.1)
					if(pressure_cnt_ > interval_time_data*10*2.5):
						self.turnOffNPSuction()
						print("!!!!!!!!!!!!Massage is time out!")
						break
				# self.intervalTime(1)
				# print("!self.interval_massage_state:",self.interval_massage_state)	
				# self.intervalTime(interval_time_data)
				# self.turnOffMainPressure()
				# return True
		else:
			print("!!!!!!!!!!!!trajectory planning failed!")
   
		# print("self.trajectory_state_ ",self.getTrajectoryState())	
		input("============ Test Press `Enter` to back to the ready point ...")

		self.turnOffNPSuction()
  
		# self.panda.read_current_pose_with_eular()
		self.ready_waypoints_ = []
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
		# print("~~Back to massage ready point!")
		# self.trajectory_state_ = 1
		ready_pose_position_state = self.panda.go_to_waypoints(self.ready_waypoints_,True,3)

		return massage_flag

	def excuteCycleMassage(self,target_pose,interval_time_data,ready_z,grasp_z):

		massage_flag = False

		self.ready_waypoints_ = []
		# self.trajectory_state_ = 0
		ready_grasp_pose1 = self.findNeighborPose(ready_z, target_pose)
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
		# print(self.ready_waypoints_)
		ready_pose_position_state = False

		# ready_grasp_pose2 = self.findNeighborPose(-0.01, target_pose)
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
		# 	self.turnOnCyclePressure(250,interval_time_data)

		self.recoveryFlagPub()
  
		self.grasp_waypoints_ = []
		deep_grasp_pose = self.findNeighborPose(grasp_z, target_pose)
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
		pressure_state = False

		if(suction_state): # planning check
			# print("self.trajectory_state_ ",self.getTrajectoryState())	

			while(self.getTrajectoryState() < 2): # trajectory running state
				
				if(self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
					print("1 !!!!!!!!!!!!Trajectory working on! getMainPressureValue ",self.getMainPressureValue())
					# sleep(0.5)
					# self.panda.move_group.stop()
					break
   
			# print("self.trajectory_state_ ",self.getTrajectoryState())	
			if(self.getTrajectoryState() == 3):
				# position_state = True
				# now1 = rospy.get_rostime()
				# sleep(5)
				# now2 = rospy.get_rostime()
    
				pressure_cnt_ = 0
				while(not self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
					pressure_cnt_ += 1
					# if pressure_cnt_%10 ==0:
					# 	print("3 pressure_cnt_:",pressure_cnt_)
					self.intervalTime(0.1)
					if(pressure_cnt_ > 50):
						print("3 !!!!!!!!!!!!Massage is time out when position is right!")
						break
  
				print("3 getMainPressureValue ",self.getMainPressureValue())
				# print("sleep time: ",now2.secs -now1.secs)
					
				# print("wpose",wpose)	
				# print("current_pose",current_pose)
			elif(self.getTrajectoryState() == 4):
				# print("Trajectory Aborted!",self.trajectory_state_)
				# position_state = False
				# sleep(5)
				pressure_cnt_ = 0
				while(not self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
					pressure_cnt_ += 1
					if pressure_cnt_%10 ==0:
						print("4 pressure_cnt_:",pressure_cnt_)
					self.intervalTime(0.1)
					if(pressure_cnt_ > 50):
						print("4 !!!!!!!!!!!!Massage is time out when Trajectory Aborted!")
						break
  
				# print("4 getMainPressureValue ",self.getMainPressureValue())

				# return False
			elif(not self.getTrajectoryState() == 1):
				print("Trajectory running problems. self.trajectory_state_: ",self.getTrajectoryState())	
				# return False


			if(self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
				# print("3 !!!!!!Pressure working on!",self.getTrajectoryState())
				pressure_state = True
				massage_flag = True
			else:
				print("3 Trajectory finished with no pressure!",self.getTrajectoryState())
				pressure_state = False
				massage_flag = False
    
    
			if(pressure_state):
				# print("!!!!!!!!!!!!Massage is working! self.interval_massage_state:",self.interval_massage_state)
				massage_flag = True
				pressure_cnt_ = 0
				while(not self.interval_massage_state):
					pressure_cnt_ += 1
					# if pressure_cnt_%10 ==0:
					# 	print("pressure_cnt_:",pressure_cnt_)
					self.intervalTime(0.1)
					if(pressure_cnt_ > interval_time_data*2*10):
						print("!!!!!!!!!!!!Massage is time out!")
						break
				# self.intervalTime(1)
				# print("!self.interval_massage_state:",self.interval_massage_state)	
				# self.intervalTime(interval_time_data)
				# self.turnOffMainPressure()
				# return True
		else:
			print("!!!!!!!!!!!!trajectory planning failed!")
   
		# print("self.trajectory_state_ ",self.getTrajectoryState())	
  
  
		# input("============ Press `Enter` to back to the ready point ...")
		# self.panda.read_current_pose_with_eular()
		self.ready_waypoints_ = []
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
		# print("~~Back to massage ready point!")
		# self.trajectory_state_ = 1
		ready_pose_position_state = self.panda.go_to_waypoints(self.ready_waypoints_,True,3)

		return massage_flag


	def excuteCycleMassageWithForce(self,target_pose,interval_time_data,ready_z,grasp_depth_force,grasp_desired_force):
		# abandon
		massage_flag = False

		print("Using translation stiffness: ", self.translation_stiffness_ )

		grasp_z = (1.00*grasp_depth_force)/self.translation_stiffness_
		print("grasp_z: ", grasp_z )
  
		print("Calculate the compensate force ratio ")
		rotation_matrix = tf.transformations.quaternion_matrix([target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w])
		
		desired_compensate_force = numpy.array([0,0,grasp_depth_force-grasp_desired_force])
		compensate_force = numpy.dot(rotation_matrix[0:3,0:3], desired_compensate_force)
		print("compensate_force " ,compensate_force)
		compensate_force_ratio = compensate_force / (self.main_valve_limit_/7.5) # positive
		self.compensate_wrench_.force.x = compensate_force_ratio[0]
		self.compensate_wrench_.force.y = compensate_force_ratio[1]
		self.compensate_wrench_.force.z = compensate_force_ratio[2]
		# self.compensate_wrench_pub.publish(self.compensate_wrench_)
		print("self.compensate_wrench_ " ,self.compensate_wrench_)
  
		self.ready_waypoints_ = []
		# self.trajectory_state_ = 0
		ready_grasp_pose1 = self.findNeighborPose(ready_z, target_pose)
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
		# print(self.ready_waypoints_)
		ready_pose_position_state = False

		# ready_grasp_pose2 = self.findNeighborPose(-0.01, target_pose)
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
		# 	self.turnOnCyclePressure(250,interval_time_data)

		self.recoveryFlagPub()
  
		self.grasp_waypoints_ = []
		deep_grasp_pose = self.findNeighborPose(grasp_z, target_pose)
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
		pressure_state = False

		if(suction_state): # planning check
			# print("self.trajectory_state_ ",self.getTrajectoryState())	

			while(self.getTrajectoryState() < 2): # trajectory running state
				
				if(self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
					print("1 !!!!!!!!!!!!Trajectory working on! getMainPressureValue ",self.getMainPressureValue())
					# sleep(0.5)
					# self.panda.move_group.stop()
					break
   
			# print("self.trajectory_state_ ",self.getTrajectoryState())	
			if(self.getTrajectoryState() == 3):
				# position_state = True
				# now1 = rospy.get_rostime()
				# sleep(5)
				# now2 = rospy.get_rostime()
    
				pressure_cnt_ = 0
				while(not self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
					pressure_cnt_ += 1
					# if pressure_cnt_%10 ==0:
					# 	print("3 pressure_cnt_:",pressure_cnt_)
					self.intervalTime(0.1)
					if(pressure_cnt_ > 50):
						print("3 !!!!!!!!!!!!Massage is time out when position is right!")
						break
  
				print("3 getMainPressureValue ",self.getMainPressureValue())
				# print("sleep time: ",now2.secs -now1.secs)
					
				# print("wpose",wpose)	
				# print("current_pose",current_pose)
			elif(self.getTrajectoryState() == 4):
				# print("Trajectory Aborted!",self.trajectory_state_)
				# position_state = False
				# sleep(5)
				pressure_cnt_ = 0
				while(not self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
					pressure_cnt_ += 1
					if pressure_cnt_%10 ==0:
						print("4 pressure_cnt_:",pressure_cnt_)
					self.intervalTime(0.1)
					if(pressure_cnt_ > 50):
						print("4 !!!!!!!!!!!!Massage is time out when Trajectory Aborted!")
						break
  
				# print("4 getMainPressureValue ",self.getMainPressureValue())

				# return False
			elif(not self.getTrajectoryState() == 1):
				print("Trajectory running problems. self.trajectory_state_: ",self.getTrajectoryState())	
				# return False


			if(self.getMainPressureValue() >= self.main_valve_trigger_pressure_):
				# print("3 !!!!!!Pressure working on!",self.getTrajectoryState())
				pressure_state = True
				massage_flag = True
			else:
				print("3 Trajectory finished with no pressure!",self.getTrajectoryState())
				pressure_state = False
				massage_flag = False
    
    
			if(pressure_state):
				# print("!!!!!!!!!!!!Massage is working! self.interval_massage_state:",self.interval_massage_state)
				massage_flag = True
				pressure_cnt_ = 0
				while(not self.interval_massage_state):
					pressure_cnt_ += 1
					# if pressure_cnt_%10 ==0:
					# 	print("pressure_cnt_:",pressure_cnt_)
					self.intervalTime(0.1)
					if(pressure_cnt_ > interval_time_data*2*10):
						print("!!!!!!!!!!!!Massage is time out!")
						break
				self.intervalTime(0.5)
				# print("!self.interval_massage_state:",self.interval_massage_state)	
				# self.intervalTime(interval_time_data)
				# self.turnOffMainPressure()
				# return True
		else:
			print("!!!!!!!!!!!!trajectory planning failed!")
   
		# print("self.trajectory_state_ ",self.getTrajectoryState())	
  
  
		# input("============ Press `Enter` to back to the ready point ...")
		# self.panda.read_current_pose_with_eular()
		self.ready_waypoints_ = []
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
		# print("~~Back to massage ready point!")
		# self.trajectory_state_ = 1
		ready_pose_position_state = self.panda.go_to_waypoints(self.ready_waypoints_,True,3)

		return massage_flag

	def rotateFromEular(self,deg_rx,deg_ry,deg_rz,q_orig):
		q_rot = Quaternion()
		DE2RA = 3.1415926 / 180

		# RPY转四元数
		q_rot = tf.transformations.quaternion_from_euler(deg_rx * DE2RA, deg_ry * DE2RA, deg_rz * DE2RA)

		# print("q_rot: ",q_rot)

		# q_orig = quaternion_from_euler(0, 0, 0)
		# # Rotate the previous pose by 180* about X
		# q_rot = quaternion_from_euler(3.14159, 0, 0)
		# q_new = quaternion_multiply(q_rot, q_orig)
		return tf.transformations.quaternion_multiply(q_rot, q_orig)


	def rotateFromEularArray(self,deg_rxyz,q_orig):
		q_rot = Quaternion()
		DE2RA = 3.1415926 / 180
  
		# RPY转四元数
		q_rot = tf.transformations.quaternion_from_euler(deg_rxyz[0] * DE2RA, deg_rxyz[1] * DE2RA, deg_rxyz[2] * DE2RA)

		# print("q_rot: ",q_rot)

		# q_orig = quaternion_from_euler(0, 0, 0)
		# # Rotate the previous pose by 180* about X
		# q_rot = quaternion_from_euler(3.14159, 0, 0)
		# q_new = quaternion_multiply(q_rot, q_orig)
		return tf.transformations.quaternion_multiply(q_rot, q_orig)



	def newPoseAfterEularArrayRotation(self,deg_rx,deg_ry,deg_rz,pose_orig): # rotate from the world frame
		q_orig = [pose_orig.orientation.x,pose_orig.orientation.y,pose_orig.orientation.z,pose_orig.orientation.w]
		q_new = self.rotateFromEular(deg_rx,deg_ry,deg_rz,q_orig)
		pose_new = Pose()
		pose_new.position = copy.deepcopy(pose_orig.position)

		pose_new.orientation.x=q_new[0]
		pose_new.orientation.y=q_new[1]
		pose_new.orientation.z=q_new[2]
		pose_new.orientation.w=q_new[3]
		
		return pose_new
	
	def newPoseAfterEularArrayRotation(self,deg_rxyz,pose_orig): # rotate from the world frame
		q_orig = [pose_orig.orientation.x,pose_orig.orientation.y,pose_orig.orientation.z,pose_orig.orientation.w]
		q_new = self.rotateFromEularArray(deg_rxyz,q_orig)
		pose_new = Pose()
		pose_new.position = copy.deepcopy(pose_orig.position)

		pose_new.orientation.x=q_new[0]
		pose_new.orientation.y=q_new[1]
		pose_new.orientation.z=q_new[2]
		pose_new.orientation.w=q_new[3]
		
		return pose_new

	def newPoseFromArrayRotation(self,compensate_position_array,deg_rxyz,pose_orig): # rotate from the world frame
		new_pose = self.newPoseAfterEularArrayRotation(deg_rxyz,pose_orig)

		new_pose.position.x = pose_orig.position.x + compensate_position_array[0]
		new_pose.position.y = pose_orig.position.y + compensate_position_array[1]
		new_pose.position.z = pose_orig.position.z + compensate_position_array[2]

		return new_pose

	def generateRotationArrayRxy(self,rx,ry,num):
		rotation_array = []
		pose_new = [0,0,0]
		for row_i in range(0,num+1):
			pose_new[0] = math.sin(2*math.pi*row_i/num)*rx
			pose_new[1]  = math.cos(2*math.pi*row_i/num)*ry
			rotation_array.append(copy.deepcopy(pose_new))
		# print("rotation_array: ",rotation_array)
		return rotation_array
		
	def generateWptsFromRotationArray(self,rot_arr,wpt):
		wpts = []
		# wpose = Pose()
		rot_arr_ = numpy.array(rot_arr)
		print("rot_arr.shape: ",rot_arr_.shape)

		(rotation_row,col) = rot_arr_.shape
		for row_i in range(0,rotation_row):
			newpose = self.newPoseFromArrayRotation([0,0,0.00],rot_arr_[row_i],wpt)
			# newpose = self.newPoseAfterEularArrayRotation(rot_arr_[row_i],wpt)
			wpts.append(copy.deepcopy(newpose))
		# print("wpts: ",wpts)

		return wpts

	def cyclicSuction(self):

		row = len(self.suction_traj_)
		# row = 1
		# br = tf.TransformBroadcaster()

		row_success_flag_cnt = 0
		
		try:
			print("Cyclic suction!")
			input("============ Press `Enter` for move to start ...")
			self.turnOffMainPressure()
			self.panda.set_joint_velocity_scale(0.2)
			self.panda.set_joint_acceleration_scale(0.2)

			# input("============ Press `Enter` to move to the start point and turn on cycle pressure ...")
			self.panda.move_to_start()
	
			
			self.panda.set_joint_velocity_scale(0.1)
			self.panda.set_joint_acceleration_scale(0.1)
			print("Record data turning on!")
			self.turnOnRecording()

			for row_i in range(0,row):
				# row_i=4
				pose = copy.deepcopy(self.suction_traj_[row_i])

				# print("suction pose num: ", row_i)
				# print("suction pose : ", pose)

				# br.sendTransform(
				# 	(pose.position.x, pose.position.y, pose.position.z),
				# 	(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
				# 	rospy.Time.now(),  
				# 	"world_norm_"+str(row_i),    
				# 	"world" 
				# 	)

				mode = input("============ Press `Enter` for move for action and 'c' for stop...")
				if mode == 'c':
					break
				# new_pose = pneumatic.newPoseFromArrayRotation([0,0,0],rotation_array[rotation_row_i],pose_from_array)
				row_success_flag = self.excuteCycleMassageWithForceControl(pose, 
					self.suction_cyclic_time_,
					self.suction_cyclic_ready_depth_,
					self.suction_cyclic_grasp_depth_,
					self.suction_desired_force_
					)

				if row_success_flag:
					row_success_flag_cnt = row_success_flag_cnt+1	

			print("********************************* massage target pose success percentage ",row_i, row_success_flag_cnt)

			self.panda.set_joint_velocity_scale(0.2)
			self.panda.set_joint_acceleration_scale(0.2)
			self.panda.set_line_velocity_scale(0.1)
			self.panda.move_to_start()

			return row_success_flag_cnt/row, True
		except rospy.ROSInterruptException:
			return 0,0
		except KeyboardInterrupt:
			return 0,0

	def broadcastFrame(self,suction_traj):

		if suction_traj == None:
			return
		row = len(suction_traj)
		
		br = tf.TransformBroadcaster()

		for row_i in range(0,row):

			pose = copy.deepcopy(suction_traj[row_i])

			br.sendTransform(
				(pose.position.x, pose.position.y, pose.position.z),
				(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
				rospy.Time.now(),  
				"world_norm_"+str(row_i),    
				"world" 
				)


	def makeSuction(self,req):
		print("Making suction!")

		self.suction_traj_ = req.suction_traj.poses
		self.suction_mode_ = req.suction_mode
		self.suction_desired_force_ = req.suction_force
		self.suction_cyclic_time_ = req.suction_time
		
		resp = SuctionPoseResponse()

		if self.suction_traj_ ==None:
			resp.success_rate =0
			resp.result = False
			rospy.logwarn('Suction trajectory is wrong!')
		
		# if self.suction_traj_ ==None:
		# 	resp.success_rate =0
		# 	resp.result = False
		# 	rospy.logwarn('Suction trajectory is wrong!')
		# 	return resp
		else:
			if self.suction_mode_ == 1:
				resp.success_rate,resp.result = self.cyclicSuction()
			elif self.suction_mode_ == 2:
				pass
			else:
				resp.success_rate =0
				resp.result = False
				rospy.logwarn('Suction mode is not work!')

		print("Response: ", resp)
		
		return resp




def main_publish_test():
	pneumatic = ArduinoPneumaticActuator()

	try:
		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			pneumatic.mainPressureVelTopicPub()
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



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

