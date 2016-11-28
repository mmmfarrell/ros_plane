#!/usr/bin/env python
# Python implementation of "path_follower"
# path_follower_base.h/cpp & path_follower_example.h/cpp

import rospy
from fcu_common.msg import FW_State, FW_Current_Path, FW_Controller_Commands
from dynamic_reconfigure.server import Server
from ros_plane.cfg import FollowerConfig
from math import *
import numpy as np

class path_follower_base:

	# Init function
	def __init__(self):
		
		# Init stuff here
		self._params.chi_infty = rospy.get_param('CHI_INFTY', 1.0472)
		self._params.k_path = rospy.get_param('K_PATH', 0.025)
		self._params.k_orbit = rospy.get_param('K_ORBIT', 8.0)

		# Dynamic Reconfigure
		self._server = Server(FollowerConfig, self.reconfigure_callback)

		# inititlialize subscribers
		self._vehicle_state_sub = rospy.Subscriber('state', FW_State, self.vehicle_state_callback)
		self._current_path_sub = rospy.Subscriber('current_path', FW_Current_Path, self.current_path_callback)

		# Init Publishers
		self.controller_commands_pub = rospy.Publisher('controller_commands', FW_Controller_Commands, queue_size=1)

		# Init Timer
		self.update_rate = 100.0
		self.update_timer_ = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update)

	# Subclasses
	class input_s:
		flag = True
		Va_d = 0.0
		r_path = np.array([0.0, 0.0, 0.0]) 
		q_path = np.array([0.0, 0.0, 0.0]) 
		c_orbit = np.array([0.0, 0.0, 0.0])
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
	_params = params_s()
	_input = input_s()

	# Callback functions
	def update(self, event):
		# print "Update (Timer Callback)"
		output = self.output_s()

		output = self.follow(self._params, self._input, output)

		msg = FW_Controller_Commands()
		msg.chi_c = output.chi_c
		msg.Va_c = output.Va_c
		msg.h_c = output.h_c
		self.controller_commands_pub.publish(msg)

	def vehicle_state_callback(self, msg):
		print "Vehicle State Callback"
		_vehicle_state = msg
		self._input.pn = _vehicle_state.position[0]
		self._input.pe = _vehicle_state.position[1]
		self._input.h = -_vehicle_state.position[2]
		self._input.chi = _vehicle_state.chi

	def current_path_callback(self, msg):
		print "Current Path Callback"
		_current_path = msg
		self._input.flag = _current_path.flag
		self._input.Va_d = _current_path.Va_d
		for i in range(0,3):
			self._input.r_path[i] = _current_path.r[i]
			self._input.q_path[i] = _current_path.q[i]
			self._input.c_orbit[i] = _current_path.c[i]
		self._input.rho_orbit = _current_path.rho
		self._input.lam_orbit = _current_path.lambda_

	def reconfigure_callback(self, config, level):
		print "Reconfigure Callback"
		self._params.chi_infty = config.CHI_INFTY
		self._params.k_path = config.K_PATH
		self._params.k_orbit = config.K_ORBIT

		# rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
		# 	{str_param}, {bool_param}, {size}""".format(**config))
		return config

	def follow(self, params, inpt, output):
		if inpt.flag:
			rospy.loginfo("Params are: \n params.k_path = %4.3f\n chi_infty = %5.4f" % (params.k_path, params.chi_infty)) # FIX THIS
			rospy.loginfo("Params are: \n params.k_orbit = %3.2f\n rho_orbit = %4.1f\n lam_orbit = %i" % (params.k_orbit, inpt.rho_orbit, inpt.lam_orbit)) # FIX THIS

			# Compute wrapped version of the path angle
			chi_q = atan2(inpt.q_path[1],inpt.q_path[0])
			while (chi_q - inpt.chi < -np.pi):
			    chi_q += 2*np.pi
			while (chi_q - inpt.chi > np.pi):
			    chi_q -= 2*np.pi

			path_error = -sin(chi_q)*(inpt.pn - inpt.r_path[0]) + cos(chi_q)*(inpt.pe - inpt.r_path[1])
			
			# heading command
			output.chi_c = chi_q - params.chi_infty*2.0/np.pi*atan(params.k_path*path_error)

			# desired altitude
			h_d = -inpt.r_path[2]-sqrt((inpt.r_path[0] - inpt.pn)**2 + (inpt.r_path[1] - inpt.pe)**2)*(inpt.q_path[2])/sqrt((inpt.q_path[0]**2) + (inpt.q_path[1]**2))
			
			# commanded altitude is desired altitude
			output.h_c = h_d
		else:
			rospy.loginfo("Params are: \n params.k_path = %4.3f \n chi_infty = %5.4f" % (params.k_path, params.chi_infty))
			rospy.loginfo("Params are: \n params.k_orbit = %3.2f \n rho_orbit = %4.1f\n lam_orbit = %i" % (params.k_orbit, inpt.rho_orbit, inpt.lam_orbit))
			d = sqrt(((inpt.pn - inpt.c_orbit[0])**2) + ((inpt.pe - inpt.c_orbit[1])**2)) # distance from orbit center
			# compute wrapped version of angular position on orbit
			varphi = atan2(inpt.pe - inpt.c_orbit[1], inpt.pn - inpt.c_orbit[0])
			while ((varphi - inpt.chi) < -np.pi):
				varphi += 2*np.pi
			while ((varphi - inpt.chi) > np.pi):
				varphi -= 2*np.pi
			# compute orbit error
			norm_orbit_error = (d - inpt.rho_orbit)/inpt.rho_orbit
			output.chi_c = varphi + inpt.lam_orbit*(np.pi/2.0 + atan(params.k_orbit*norm_orbit_error))

			# commanded altitude is the height of the orbit
			h_d = -inpt.c_orbit[2]
			output.h_c = h_d
		output.Va_c = inpt.Va_d
		return output

##############################
#### Main Function to Run ####
##############################
if __name__ == '__main__':
	
	# Initialize Node
	rospy.init_node('ros_plane_follower')

	# init path_follower object
	manager = path_follower_base()

	rospy.spin()
