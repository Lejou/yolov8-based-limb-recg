#!/usr/bin/env python3

from __future__ import print_function
# from asyncio import current_task

import copy
# from re import S

from time import sleep
from franka_msgs import msg
from genpy import Time

import rospy
from six.moves import input
from std_msgs.msg import String,Int16,Float32,Float32MultiArray,Bool
from control_msgs.msg import FollowJointTrajectoryActionFeedback,FollowJointTrajectoryActionResult
from geometry_msgs.msg import WrenchStamped,Quaternion
import tf.transformations 

import numpy

from geometry_msgs.msg import Quaternion, Pose, Point, Wrench
from transforms3d.quaternions import quat2mat, mat2quat
import roslaunch
import threading 

import PandaMoveGroup


def main():

	try:
		print("Press Ctrl-D to exit at any time")
		input(
			"============ 1 Press `Enter` to move to the start point ..."
		)
		# pneumatic.panda.move_to_start()

		input(
			"============ 3 Press `Enter` to move to the start point ..."
		)
		# pneumatic.panda.move_to_start()
		print("============ Python tutorial demo complete!")
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == "__main__":
	main()

