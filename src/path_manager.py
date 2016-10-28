#!/usr/bin/env python
# Python implementation of "path_manager"
# path_manager_base.h

import rospy
from fcu_common.msg import FW_State, FW_Current_Path, FW_Waypoint
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import Float32, Float32MultiArray
import math
#import Eigen
#from ros_plane import ControllerConfig

SIZE_WAYPOINT_ARRAY = 20

class path_manager_base:

	# Init function
	def __init__(self):
		# R_min param

		# inititlialize subscribers
		self._vehicle_state_sub = rospy.Subscriber('state', FW_State, self.vehicle_state_callback)
		self._new_waypoint_sub = rospy.Subscriber('waypoint_path', FW_Waypoint, self.new_waypoint_callback)

		# Init Publishers
		self._current_path_pub = rospy.Subscriber('current_path', FW_Current_Path, queue_size=10)

	# Class members
	_num_waypoints = 0
	_vehicle_state = FW_State()
	_waypoints = [waypoint_s() for _ in range(SIZE_WAYPOINT_ARRAY)]

	# Subclasses
	class waypoint_s:
		w = [0.0, 0.0, 0.0]
		chi_d = 0.0
		chi_valid = True
		Va_d = 0.0


 	class input_s:
	 	pn = 0.0 # position North
	 	pe = 0.0 # position East
	 	h = 0.0 # Altitude
	 	chi = 0.0 # course angle

 	class output_s:
	 	flag = True # Inicates strait line or orbital path (true is line, false is orbit)
	 	Va_d = 0.0 # Desired airspeed (m/s)
	 	r = [] # Vector to origin of straight line path (m)
	 	q = [] # Unit vector, desired direction of travel for line path
	 	c = [] # Center of orbital path (m)
	 	rho = 0.0 # Radius of orbital path (m)
	 	lambdaa = 1 # Direction of orbital path (cw is 1, ccw is -1)

 	class param_s:
 		R_min = 0.0


 	# Class Member Functions
	def waypoint_init(self):
		_waypoints[_num_waypoints].w[0]  	 = 0
		_waypoints[_num_waypoints].w[1]      = 0
		_waypoints[_num_waypoints].w[2]      = -100
		_waypoints[_num_waypoints].chi_d     = -9999
		_waypoints[_num_waypoints].chi_valid = 0
		_waypoints[_num_waypoints].Va_d      = 35
		_num_waypoints+=1

		_waypoints[_num_waypoints].w[0]      = 1000
		_waypoints[_num_waypoints].w[1]      = 0
		_waypoints[_num_waypoints].w[2]      = -100
		_waypoints[_num_waypoints].chi_d     = -9999
		_waypoints[_num_waypoints].chi_valid = 0
		_waypoints[_num_waypoints].Va_d      = 35
		_num_waypoints+=1

		_waypoints[_num_waypoints].w[0]      = 1000
		_waypoints[_num_waypoints].w[1]      = 1000
		_waypoints[_num_waypoints].w[2]      = -100
		_waypoints[_num_waypoints].chi_d     = -9999
		_waypoints[_num_waypoints].chi_valid = 0
		_waypoints[_num_waypoints].Va_d      = 35
		_num_waypoints+=1

		_waypoints[_num_waypoints].w[0]      = 0
		_waypoints[_num_waypoints].w[1]      = 1000
		_waypoints[_num_waypoints].w[2]      = -100
		_waypoints[_num_waypoints].chi_d     = -9999
		_waypoints[_num_waypoints].chi_valid = 0
		_waypoints[_num_waypoints].Va_d      = 35
		_num_waypoints+=1

		_waypoints[_num_waypoints].w[0]      = 0
		_waypoints[_num_waypoints].w[1]      = 0
		_waypoints[_num_waypoints].w[2]      = 0
		_waypoints[_num_waypoints].chi_d     = -9999
		_waypoints[_num_waypoints].chi_valid = 0
		_waypoints[_num_waypoints].Va_d      = 35
		_num_waypoints+=1

  	def vehicle_state_callback(self, msg):
		self._vehicle_state = msg
		inpt = self.input_s()
		inpt.pn = self._vehicle_state.position[0]
		inpt.pe = self._vehicle_state.position[1]
		inpt.h = -self._vehicle_state.position[2]
		inpt.chi = self._vehicle_state.chi

		outputs = self.output_s()
		params = self.params_s()
		# manage(params, input, outputs)
		# self.current_path_publisher(outputs)

  	def new_waypoint_callback(self, msg):
		self._waypoints[self._num_waypoints].w[0]      = msg.w[0]
		self._waypoints[self._num_waypoints].w[1]      = msg.w[1]
		self._waypoints[self._num_waypoints].w[2]      = msg.w[2]
		self._waypoints[self._num_waypoints].chi_d     = msg.chi_d
		self._waypoints[self._num_waypoints].chi_valid = msg.chi_valid
		self._waypoints[self._num_waypoints].Va_d      = msg.Va_d
		self._num_waypoints+=1

	def current_path_publsih(self, output):
		current_path = FW_Current_Path()

		current_path.flag = output.flag
		current_path.Va_d = output.Va_d

		for i in range(0,3):
			current_path.r[i] = output.r[i]
			current_path.q[i] = output.q[i]
			current_path.c[i] = output.c[i]

		current_path.rho = output.rho
		current_path.lambdaa = output.lambdaa

		_current_path_pub.publish(current_path)
