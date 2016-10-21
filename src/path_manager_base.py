#!/usr/bin/env python
# Python implementation of "path_manager_base"
# path_manager_base.cpp

import path_manager.py as rosplane
import rospy as ros

rosplane.path_manager_base()


##############################
#### Main Function to Run ####
##############################
if __name__ == '__main__':
	# Initialize Node
	ros.init_node('ros_plane_path_manager')
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
	ros.spin()