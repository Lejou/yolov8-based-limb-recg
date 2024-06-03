#!/usr/bin/env python3

from __future__ import print_function
from asyncio import current_task

import copy
import rospy
import PandaMassage
from time import sleep
import numpy

def main():

	pneumatic = PandaMassage.ArduinoPneumaticActuator()

	# wpose = pneumatic.panda.read_current_pose()
	c =''
	input("============ Press `Enter` to record point ...")
	try:
		while(not c == 'c'):
			wpose = pneumatic.panda.read_current_pose()
			c= input("")

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == "__main__":
	main()

