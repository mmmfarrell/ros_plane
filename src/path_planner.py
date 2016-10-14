#!/usr/bin/env python
# Python implementation of "path_planner.cpp"

import rospy
from fcu_common.msg import FW_Waypoint
import math

num_waypoints = 4

def talker():
	rospy.init_node('ros_plane_path_planner', anonymous=True)
	
	waypointPublisher = rospy.Publisher('waypoint_path',FW_Waypoint, queue_size=10)
	
	d = rospy.Duration(.5) 
	rospy.sleep(d)

	Va = 8.5 # 11.0
	wps =  [
                -10, -10, -30, -45, Va,
                -10, -125, -30, -135*math.pi/180, Va,
                -125, -10, -30, 45*math.pi/180, Va,
                -125, -125, -30, 135*math.pi/180, Va]
	for i in range(0,num_waypoints):

		new_waypoint = FW_Waypoint()
		
		new_waypoint.w[0] = wps[i*5 + 0]
		new_waypoint.w[1] = wps[i*5 + 1]
		new_waypoint.w[2] = wps[i*5 + 2]
		new_waypoint.chi_d = wps[i*5 + 3]
		
		new_waypoint.chi_valid = True # False
		new_waypoint.Va_d = wps[i*5 + 4]
		
		waypointPublisher.publish(new_waypoint)
		
		d = rospy.Duration(0.5)
		rospy.sleep(d)
						

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
