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

	home_point = [0.6075, -0.1440, 0.4685, 0.9997762807009487, -0.00797496962048781, -0.005010525786338952, 0.01893892918500345]
	# home_point = [0.48867849376068756, -0.12503557910295854, 0.4281667601764035, 0.9997762807009487, -0.00797496962048781, -0.005010525786338952, 0.01893892918500345]

	pneumatic.panda.set_start_point(home_point)
 
	pneumatic.turnOffNPSuction()

	rate = rospy.Rate(20)

	try:
		while not rospy.is_shutdown():

			pneumatic.broadcastFrame(pneumatic.suction_traj_)

			rate.sleep() 
			
	except rospy.ROSInterruptException:
		pass


if __name__ == "__main__":
	main()

