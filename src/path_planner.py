#!/usr/bin/env python
# Python implementation of "path_planner.cpp"

import rospy
from fcu_common.msg import FW_Waypoint
import math

num_waypoints = 5

def publishwaypoints():

	# Init ROS Node
	rospy.init_node('ros_plane_path_planner', anonymous=True)
	
	# Init Publisher
	waypointPublisher = rospy.Publisher('/waypoint_path',FW_Waypoint, queue_size=10)
	
	# Sleep, (this fixed bug of first waypoint not publishing)
	d = rospy.Duration(.5) 
	rospy.sleep(d)

	# Set waypoints
	Va = 30.0#8.5 # 11.0
	wps =  [
				500, 0, -100, 0, Va,
				1000, 250, -100, 0, Va,
				1500, 500, -100, 0, Va,
				0, -500, -100, 0, Va]
                # -10, -10, -30, -45, Va,
                # -10, -125, -30, -135*math.pi/180, Va,
                # -125, -10, -30, 45*math.pi/180, Va,
                # -125, -125, -30, 135*math.pi/180, Va]
	
    # Loop through each waypoint
	for i in range(0,num_waypoints):

		# Make waypoint a FW_Waypoint msg
		new_waypoint = FW_Waypoint()
		
		new_waypoint.w[0] = wps[i*5 + 0]
		new_waypoint.w[1] = wps[i*5 + 1]
		new_waypoint.w[2] = wps[i*5 + 2]
		new_waypoint.chi_d = wps[i*5 + 3]
		
		new_waypoint.chi_valid = True # True
		new_waypoint.Va_d = wps[i*5 + 4]
		
		# Publish the Waypoint
		waypointPublisher.publish(new_waypoint)
		
		# Sleep
		d = rospy.Duration(0.5)
		rospy.sleep(d)
						

if __name__ == '__main__':

	# Just run the publisher once
	try:
		publishwaypoints()
	except rospy.ROSInterruptException:
		pass
