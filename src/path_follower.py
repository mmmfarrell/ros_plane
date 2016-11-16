#!/usr/bin/env python
# Python implementation of "path_follower"
# path_follower_base.h/cpp & path_follower_example.h/cpp

import rospy
from fcu_common.msg import FW_State, FW_Current_Path, FW_Controller_Commands
# dynamic reconfigure
# ros_plane follower config
from math import *
import numpy as np

class path_follower_base:

	# Init function
	def __init__(self):
		
		# Init stuff here

		# inititlialize subscribers
		self._vehicle_state_sub = rospy.Subscriber('state', FW_State, self.vehicle_state_callback)
		self._new_waypoint_sub = rospy.Subscriber('current_path', FW_Current_Path, self.current_path_callback)

		# Init Publishers
		self._current_path_pub = rospy.Publisher('controller_commands', FW_Controller_Commands, queue_size=1)

	# Subclasses
	class input_s:
		flag = True
		Va_d = 0.0
		r_path = np.array([0.0, 0.0, 0.0]) 
		q_path = np.array([0.0, 0.0, 0.0]) 
		c_path = np.array([0.0, 0.0, 0.0])
		rho_orbit = 0.0
		lam_orbit = 1
		pn = 0.0 			# position north
		pe = 0.0			# position east
		h = 0.0				# altitude
		Va = 0.0			# airspeed
		chi = 0.0			# course angle

	class output_s:
		Va_c = 0.0 			# commanded airspeed (m/s)
		h_c = 0.0			# commanded altitude (m)
		chi_c = 0.0 		# commanded course (rad)

	class params_s:
		chi_infty = 0.0
		k_path = 0.0
		k_orbit = 0.0 

	# Class Members
	update_rate = 100.0
	update_timer_ = rospy.Timer()