#!/usr/bin/env python
# Python implementation of "path_planner.cpp"

import rospy
from fcu_common.msg import FW_Waypoint
import math

num_waypoints = 4

def talker():
	rospy.init_node('ros_plane_path_planner', anonymous=True)
	
	waypointPublisher = rospy.Publisher('waypoint_path',FW_Waypoint, queue_size=10)
	
	Va = 8.5 # 11.0
	wps[5*num_waypoints] =  {
                -10, -10, -30, -45, Va,
                -10, -125, -30, -135*math.pi/180, Va,
                -125, -10, -30, 45*math.pi/180, Va,
                -125, -125, -30, 135*math.pi/180, Va}
	print(wps)

talker()
