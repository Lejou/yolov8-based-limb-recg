#!/usr/bin/env python3

from __future__ import print_function
from asyncio import current_task

import copy
from re import S
import sys
from math import cos, dist, fabs, pi, tau


import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotTrajectory
from six.moves import input

from trajectory_msgs.msg import JointTrajectoryPoint
import tf.transformations


from transforms3d.quaternions import quat2mat, mat2quat

def all_close(goal, actual, tolerance):
	"""
	Convenience method for testing if the values in two lists are within a tolerance of each other.
	For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
	between the identical orientations q and -q is calculated correctly).
	@param: goal	   A list of floats, a Pose or a PoseStamped
	@param: actual	 A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
		x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
		# Euclidean distance
		d = dist((x1, y1, z1), (x0, y0, z0))
		# phi = angle between orientations
		cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
		return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

	return True


class MoveGroupPanda(object):
	"""MoveGroupPanda"""

	def __init__(self):
		super(MoveGroupPanda, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node("move_group_panda", anonymous=True)
  
		self.robot = moveit_commander.RobotCommander()

		self.scene = moveit_commander.PlanningSceneInterface()  

		groups = self.robot.get_group_names()
 
		# 打印所有move group的名字
		print("Available move groups:")
		for group in groups:
			print(group)

		group_name = "panda_arm"
		self.move_group = moveit_commander.MoveGroupCommander(group_name)    
		
 
		# # set planning pipeline
		# # Available planers ompl, chomp, stomp
		# mgi.move_group.set_planning_pipeline_id('ompl')

		# # set planner id, see {planner}_planning_pipeline.launch available options
		# # mgi.move_group.set_planner_id('RRTConnect')
 

		# self.move_group.set_planner_id("CHMOP")
		self.move_group.set_planning_pipeline_id("ompl")
		self.move_group.set_planning_time(2)
		self.move_group.set_num_planning_attempts(10)
  
		# self.move_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
		# self.move_group.set_planner_id("LIN")
		# self.move_group.set_planning_pipeline_id("ompl")
		# self.move_group.set_planner_id("RRTstar")
  
		## Create a `DisplayTrajectory`_ ROS publisher which is used to display
		## trajectories in Rviz:
		display_trajectory_publisher = rospy.Publisher(
			"/move_group/display_planned_path",
			moveit_msgs.msg.DisplayTrajectory,
			queue_size=20,
		)

		# We can get the name of the reference frame for this robot:
		planning_frame = self.move_group.get_planning_frame()
		# print("============ Planning frame: %s" % planning_frame)

		# We can also print the name of the end-effector link for this group:
		self.move_group.set_end_effector_link("/probe_ee")
		# self.move_group.set_end_effector_link("panda_link8")
		eef_link = self.move_group.get_end_effector_link()
		print("============ End effector link: %s" % eef_link)
		
		# We can get a list of all the groups in the robot:
		# group_names = robot.get_group_names()
		# print("============ Available Planning Groups:", robot.get_group_names())

		# pose_goal = geometry_msgs.msg.Pose()
		# link8
		# pose_goal.position.x = 0.306
		# pose_goal.position.y = 0.0
		# pose_goal.position.z = 0.55
  
  
		# probe_ee 0
		# pose_goal.position.x = 0.311
		# pose_goal.position.y = -0.006
		# pose_goal.position.z = 0.506
		# pose_goal.orientation.x = 1
		# pose_goal.orientation.y = 0
		# pose_goal.orientation.z = 0.0
		# pose_goal.orientation.w = 0.0

		# probe_ee 1
		# pose_goal.position.x = 0.325
		# pose_goal.position.y = -0.03
		# pose_goal.position.z = 0.354
		# pose_goal.orientation.x = -0.892376556791906
		# pose_goal.orientation.y = 0.4511411337048661
		# pose_goal.orientation.z = 0.008241541329095886
		# pose_goal.orientation.w = 0.008138632026473505

		point = [0.3244973091403415, -0.0026676111979773653, 0.5885575265056144, -0.9260554958644964, 0.3771234657570177, 0.010262797185633893, 0.009685423520767775]

		self.set_start_point(point)

		# Misc variables
		# self.robot = robot
		# self.scene = scene
		# self.move_group = move_group
		# self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_name
		self.velocity_scale = 0.1
		self.acceleration_scale =0.1
		# self.start_pose = pose_goal

	def go_to_home_joint(self):
		move_group = self.move_group

		joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = -pi / 4
		joint_goal[2] = 0
		joint_goal[3] = -pi *3/ 4
		joint_goal[4] = 0
		joint_goal[5] = pi / 2 
		joint_goal[6] = pi / 4

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		move_group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		move_group.stop()

		## END_SUB_TUTORIAL

		# For testing:
		current_joints = move_group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)

	def go_to_pose(self):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		move_group = self.move_group

		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 1.0
		pose_goal.position.x = 0.4
		pose_goal.position.y = 0.1
		pose_goal.position.z = 0.4


		move_group.set_pose_target(pose_goal)
		
		success = move_group.go(wait=True)
		move_group.stop()
		move_group.clear_pose_targets()

		current_pose = self.move_group.get_current_pose().pose
		return all_close(pose_goal, current_pose, 0.01)
    
	def go_to_command_arm_joint(self, joint1,joint2,joint3, joint4,joint5,joint6,joint7):
		## Planning to a Joint Goal
		move_group = self.move_group
		joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = joint1
		joint_goal[1] = joint2
		joint_goal[2] = joint3
		joint_goal[3] = joint4
		joint_goal[4] = joint5
		joint_goal[5] = joint6
		joint_goal[6] = joint7

		move_group.go(joint_goal, wait=True)
		move_group.stop()
		current_joints = move_group.get_current_joint_values()
		print('current_joints',current_joints)
		print('joint_goal',joint_goal)
		return all_close(joint_goal, current_joints, 0.01)

	def execute_with_limit(self,plan,state):
		plan_temp=copy.deepcopy(plan)
		new_plan = self.move_group.retime_trajectory(self.move_group.get_current_state(),
                                          		plan_temp,
                                            	self.velocity_scale,
                                             	self.acceleration_scale,
												# 'iterative_time_parameterization')
												# 'iterative_spline_parameterization')
                                              	'time_optimal_trajectory_generation')
		# plan_temp=copy.deepcopy(plan)
		# new_plan= self.scale_trajectory_speed(plan_temp, self.line_velocity_scale)
		return self.move_group.execute(new_plan, wait=state)

	def set_start_point(self,point):
		pose_goal = geometry_msgs.msg.Pose()
		# probe_ee 1
		pose_goal.position.x = point[0]
		pose_goal.position.y = point[1]
		pose_goal.position.z = point[2]
		pose_goal.orientation.x = point[3]
		pose_goal.orientation.y = point[4]
		pose_goal.orientation.z = point[5]
		pose_goal.orientation.w = point[6]

		self.start_pose = pose_goal

	def move_to_start(self):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		# move_group = self.move_group
		pose_goal = self.start_pose
		# print("pose_goal: ",pose_goal)

		waypoints = []
		# wpose = self.move_group.get_current_pose().pose

		# waypoints.append(copy.deepcopy(wpose))
		waypoints.append(copy.deepcopy(pose_goal))
  
		loop_times = 0
		fraction = 0
		while(fraction<1):
			(plan, fraction) = self.move_group.compute_cartesian_path(
											waypoints,   # waypoints to follow
											0.01,        # eef_step
											3)         # jump_threshold
			# print("loop_times: ", loop_times)
			
			# loop_times +=1
			# if(loop_times ==1):
			# 	print("fraction: ", fraction)
				# return False

		# plan_temp=copy.deepcopy(plan)
		# new_plan= self.scale_trajectory_speed(plan_temp, self.line_velocity_scale)
		return self.execute_with_limit(plan, True)    
		# return self.execute_with_limit(plan, False)    

		# move_group.set_pose_target(pose_goal)
		# success = move_group.go(wait=True)
		# move_group.stop()
		# move_group.clear_pose_targets()
		# current_pose = self.move_group.get_current_pose().pose
		# return all_close(pose_goal, current_pose, 0.01)

	def go_to_pose_goal(self,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w):
		move_group = self.move_group
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.x = x
		pose_goal.position.y =y
		pose_goal.position.z = z

		pose_goal.orientation.x =orientation_x
		pose_goal.orientation.y = orientation_y
		pose_goal.orientation.z = orientation_z
		pose_goal.orientation.w = orientation_w

		
		move_group.set_pose_target(pose_goal)

		## Now, we call the planner to compute the plan and execute it.
		plan = move_group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		move_group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		move_group.clear_pose_targets()
		current_pose = self.move_group.get_current_pose().pose
		return all_close(pose_goal, current_pose, 0.01)

	def go_to_waypoints_without_loop_times(self,waypoints,wait_bool):
		# move_group = self.move_group
		(plan, fraction) = self.move_group.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,        # eef_step
										3)         # jump_threshold
		# print("fraction: ", fraction)
		if(fraction < 1):
			return False

		return self.execute_with_limit(plan, wait_bool)  

	# def go_to_point(self,waypoints,wait_bool):
	# 	# move_group = self.move_group
	# 	(plan, fraction) = self.move_group.compute_cartesian_path(
	# 									waypoints,   # waypoints to follow
	# 									0.01,        # eef_step
	# 									5)         # jump_threshold
	# 	# print("fraction: ", fraction)
	# 	if(fraction < 1):
	# 		return False

	# 	return self.execute_with_limit(plan, wait_bool)  

	def go_to_waypoints(self,waypoints,wait_bool,plan_loop_times):
		# move_group = self.move_group
		loop_times = 0
		fraction = 0
		while(fraction<1):
			(plan, fraction) = self.move_group.compute_cartesian_path(
											waypoints,   # waypoints to follow
											0.01,        # eef_step
											3)         # jump_threshold
			# print("loop_times: ", loop_times)
			
			loop_times +=1
			if(loop_times ==plan_loop_times):
				print("fraction: ", fraction)
				return False

		return self.execute_with_limit(plan, wait_bool)  
		# plan_temp=copy.deepcopy(plan)
		# new_plan= self.scale_trajectory_speed(plan_temp, self.line_velocity_scale)
		# return move_group.execute(new_plan, wait=wait_bool)    

	def go_to_muti_line_movement(self,muti_point,state):
		# move_group = self.move_group
		waypoints = []
		muti_point_number=len(muti_point)
		wpose = self.move_group.get_current_pose().pose
		for  i in range(muti_point_number):
			wpose.position.x=muti_point[i][0]
			wpose.position.y=muti_point[i][1]
			wpose.position.z=muti_point[i][2]
			wpose.orientation.x=muti_point[i][3]
			wpose.orientation.y=muti_point[i][4]
			wpose.orientation.z=muti_point[i][5]
			wpose.orientation.w=muti_point[i][6]
			
			waypoints.append(copy.deepcopy(wpose))
		print("waypoints: ", waypoints)

		(plan, fraction) = self.move_group.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,        # eef_step
										5)         # jump_threshold
		print("fraction: ", fraction)
		return self.execute_with_limit(plan, state)     


	def set_joint_velocity_scale(self, velocity_scale):
		self.velocity_scale=velocity_scale
		# self.move_group.set_max_velocity_scaling_factor(velocity_scale)

	def set_joint_acceleration_scale(self,acceleration_scale):
		self.acceleration_scale=acceleration_scale
		# self.move_group.set_max_acceleration_scaling_factor(acceleration_scale)

	def set_line_velocity_scale(self, line_velocity_scale):
		self.line_velocity_scale=line_velocity_scale

	def scale_trajectory_speed(self,traj, scale):
		# Create a new trajectory object
		new_traj = RobotTrajectory()

		# Initialize the new trajectory to be the same as the input trajectory
		new_traj.joint_trajectory = traj.joint_trajectory

		# Get the number of joints involved
		n_joints = len(traj.joint_trajectory.joint_names)

		# Get the number of points on the trajectory
		n_points = len(traj.joint_trajectory.points)

		# Store the trajectory points
		points = list(traj.joint_trajectory.points)
			
		# Cycle through all points and joints and scale the time from start,
		# as well as joint speed and acceleration
		for i in range(n_points):
			point = JointTrajectoryPoint()

			# The joint positions are not scaled so pull them out first
			point.positions = traj.joint_trajectory.points[i].positions

			# Next, scale the time_from_start for this point
			point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale

			# Get the joint velocities for this point
			point.velocities = list(traj.joint_trajectory.points[i].velocities)

			# Get the joint accelerations for this point
			point.accelerations = list(traj.joint_trajectory.points[i].accelerations)

			# Scale the velocity and acceleration for each joint at this point
			for j in range(n_joints):
				point.velocities[j] = point.velocities[j] * scale
				point.accelerations[j] = point.accelerations[j] * scale * scale

			# Store the scaled trajectory point
			points[i] = point

		# Assign the modified points to the new trajectory
		new_traj.joint_trajectory.points = points

		# Return the new trajecotry
		return new_traj

	def set_trajectory_speed(self, traj, speed):
		# Create a new trajectory object
		new_traj = RobotTrajectory()
		
		# Initialize the new trajectory to be the same as the input trajectory
		new_traj.joint_trajectory = traj.joint_trajectory
		
		# Get the number of joints involved
		n_joints = len(traj.joint_trajectory.joint_names)
		
		# Get the number of points on the trajectory
		n_points = len(traj.joint_trajectory.points)
			
		# Store the trajectory points
		points = list(traj.joint_trajectory.points)
		
		# Cycle through all points and joints and scale the time from start,
		# as well as joint speed and acceleration
		for i in range(n_points):
			point = JointTrajectoryPoint()
			
			# The joint positions are not scaled so pull them out first
			point.positions = traj.joint_trajectory.points[i].positions

			# Next, scale the time_from_start for this point
			point.time_from_start = traj.joint_trajectory.points[i].time_from_start
			
			# Initialize the joint velocities for this point
			point.velocities = [speed] * n_joints
			
			# Get the joint accelerations for this point
			point.accelerations = [speed / 4.0] * n_joints
			
			# Store the scaled trajectory point
			points[i] = point

		# Assign the modified points to the new trajectory
		new_traj.joint_trajectory.points = points

		# Return the new trajecotry
		return new_traj

	def execute_plan(self, plan):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		# move_group = self.move_group

		## BEGIN_SUB_TUTORIAL execute_plan
		##
		## Executing a Plan
		## ^^^^^^^^^^^^^^^^
		## Use execute if you would like the robot to follow
		## the plan that has already been computed:
		return self.execute_with_limit(plan, True)  

		## **Note:** The robot's current joint state must be within some tolerance of the
		## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
		## END_SUB_TUTORIAL

	def read_current_pose(self):
        	#get current pose
		#raw_input("stop0")
		end_effector_link = self.move_group.get_end_effector_link()
		#print("end_effector_link",end_effector_link)
		#raw_input("stop")
		x = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.position.x
		y = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.position.y
		z = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.position.z

		rx = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.x
		ry = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.y
		rz = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.z
		rw = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.w
		#print("start_pose",start_pose)
		
		current_pose=[x,y,z,rx,ry,rz,rw]
		# print("current_pose",current_pose)
		print(current_pose,",")
		return current_pose

	def read_current_pose_with_eular(self):
        	#get current pose
		#raw_input("stop0")
		end_effector_link = self.move_group.get_end_effector_link()
		#print("end_effector_link",end_effector_link)
		#raw_input("stop")
		x = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.position.x
		y = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.position.y
		z = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.position.z

		rx = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.x
		ry = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.y
		rz = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.z
		rw = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.w
  
		(eular_rx,eular_ry,eular_rz) = tf.transformations.euler_from_quaternion([rx,ry,rz,rw])

		rad2degree = 180/3.1415926
		#print("start_pose",start_pose)
		
		current_pose=[x,y,z,eular_rx*rad2degree,eular_ry*rad2degree,eular_rz*rad2degree]
		print("current_pose",current_pose)
		# print(current_pose,",")
		return current_pose

	def read_current_joints(self):
    	#get current joints
		current_joints = self.move_group.get_current_joint_values()
		return current_joints

	# def rotate_EE(self,rx,ry,rz):
	# 	[x,y,z,current_rx,current_ry,current_rz] = self.read_current_pose()
	# 	rotate_pose_rx = current_rx+rx
	# 	rotate_pose_ry = current_ry+ry
	# 	rotate_pose_rz = current_rz+rz
	# 	rotate_pose = [x,y,z,rotate_pose_rx,rotate_pose_ry,rotate_pose_rz]

	# 	return
