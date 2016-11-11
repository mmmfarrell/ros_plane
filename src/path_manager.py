#!/usr/bin/env python
# Python implementation of "path_manager"
# path_manager_base.h & path_manager_base.py

import rospy
from fcu_common.msg import FW_State, FW_Current_Path, FW_Waypoint
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import Float32, Float32MultiArray
from math import *
import numpy as np
#import Eigen
#from ros_plane import ControllerConfig

# Set size of waypoint array (max # of waypoints)
SIZE_WAYPOINT_ARRAY = 20

# These were global constants in the cpp files, can change to np.pi and np.pi/2 
M_PI_F = 3.14159265358979323846
M_PI_2_F = 1.57079632679489661923

class path_manager_base:

	# Init function
	def __init__(self):
		print 'Base Init'

		# Init Params
		self.params.R_min = rospy.get_param('R_min',100.0)

		# run waypoint init to initialize with waypoints found in waypoint_init (alternative to path_planner.py)
		# self.waypoint_init()

		# inititlialize subscribers
		self._vehicle_state_sub = rospy.Subscriber('state', FW_State, self.vehicle_state_callback)
		self._new_waypoint_sub = rospy.Subscriber('waypoint_path', FW_Waypoint, self.new_waypoint_callback)

		# Init Publishers
		self._current_path_pub = rospy.Publisher('current_path', FW_Current_Path, queue_size=10)

	# Subclasses
	class waypoint_s:
		w = [0.0, 0.0, 0.0] # position NED
		chi_d = 0.0 # heading at waypoint
		chi_valid = True # False = don't worry about heading at waypoint
		Va_d = 0.0 # airspeed at waypoint

		def __str__(self):
			return "w: " + str(self.w) + "\nchi_d: " + str(self.chi_d) + "\nchi_valid: " + str(self. chi_valid) + "\nVa_d: " + str(self.Va_d)

	# I had problems with an array of instances of waypoint_s because of the w array, so here's my temporary fix
	class waypoint_temp:
		w0 = 0.0
		w1 = 0.0
		w2 = 0.0
		chi_d = 0.0
		chi_valid = True
		Va_d = 0.0

		def __str__(self):
			return "w: " + str([self.w0,self.w1,self.w2]) + "\nchi_d: " + str(self.chi_d) + "\nchi_valid: " + str(self. chi_valid) + "\nVa_d: " + str(self.Va_d)

 	class input_s:
	 	pn = 0.0 # position North
	 	pe = 0.0 # position East
	 	h = 0.0 # Altitude
	 	chi = 0.0 # course angle

 	# output for current_path
 	class output_s:
	 	flag = True # Inicates strait line or orbital path (true is line, false is orbit)
	 	Va_d = 0.0 # Desired airspeed (m/s)
	 	r = [0.0, 0.0, 0.0] # Vector to origin of straight line path (m)
	 	q = [0.0, 0.0, 0.0] # Unit vector, desired direction of travel for line path
	 	c = [0.0, 0.0, 0.0] # Center of orbital path (m)
	 	rho = 0.0 # Radius of orbital path (m)
	 	lambda_ = 1 # Direction of orbital path (cw is 1, ccw is -1)

 	class params_s:
 		R_min = 0.0 # Minimum turning radius

	# Class members
	_num_waypoints = 0
	_vehicle_state = FW_State()
	fillet_state = 'Straight' # 'Straight' or 'Orbit'
	dubin_state = 'First' #'First', 'Before_H1', 'Before_H1_wrong_side', 'Straight', 'Before_H3', 'Before_H3_wrong_side'
	_waypoints = [waypoint_temp() for _ in range(SIZE_WAYPOINT_ARRAY)]
	params = params_s()


 	# Class Member Functions
	def waypoint_init(self):
		print 'Waypoint Init'
		self._waypoints[self._num_waypoints].w0  	 = 0
		self._waypoints[self._num_waypoints].w1      = 0
		self._waypoints[self._num_waypoints].w2      = -100
		self._waypoints[self._num_waypoints].chi_d     = 0.0#-9992
		self._waypoints[self._num_waypoints].chi_valid = True
		self._waypoints[self._num_waypoints].Va_d      = 35
		self._num_waypoints+=1

		self._waypoints[self._num_waypoints].w0      = 1000
		self._waypoints[self._num_waypoints].w1      = 0
		self._waypoints[self._num_waypoints].w2      = -100
		self._waypoints[self._num_waypoints].chi_d     = 0.5#-9993
		self._waypoints[self._num_waypoints].chi_valid = True
		self._waypoints[self._num_waypoints].Va_d      = 35
		self._num_waypoints+=1

		self._waypoints[self._num_waypoints].w0      = 1000
		self._waypoints[self._num_waypoints].w1      = 1000
		self._waypoints[self._num_waypoints].w2      = -100
		self._waypoints[self._num_waypoints].chi_d     = -0.5#-9994
		self._waypoints[self._num_waypoints].chi_valid = True
		self._waypoints[self._num_waypoints].Va_d      = 35
		self._num_waypoints+=1

		self._waypoints[self._num_waypoints].w0      = 0
		self._waypoints[self._num_waypoints].w1      = 1000
		self._waypoints[self._num_waypoints].w2      = -100
		self._waypoints[self._num_waypoints].chi_d     = 0.0#-9995
		self._waypoints[self._num_waypoints].chi_valid = True
		self._waypoints[self._num_waypoints].Va_d      = 35
		self._num_waypoints+=1

		self._waypoints[self._num_waypoints].w0      = -1000
		self._waypoints[self._num_waypoints].w1      = -1000
		self._waypoints[self._num_waypoints].w2      = -100
		self._waypoints[self._num_waypoints].chi_d     = 0.75#-9996
		self._waypoints[self._num_waypoints].chi_valid = True
		self._waypoints[self._num_waypoints].Va_d      = 35
		self._num_waypoints+=1

		# self.waypointprint()

  	def vehicle_state_callback(self, msg):
		print 'Vehicle State Callback'
		self._vehicle_state = msg
		inpt = self.input_s() # set inpt to _vehicle_state
		inpt.pn = self._vehicle_state.position[0]
		inpt.pe = self._vehicle_state.position[1]
		inpt.h = -self._vehicle_state.position[2]
		inpt.chi = self._vehicle_state.chi

		outputs = self.output_s()
		outputs = self.manage(self.params, inpt, outputs) 		# call manager
		self.current_path_publisher(outputs) 	# publish current_path from outputs

  	def new_waypoint_callback(self, msg):
  		print 'New Waypoint Callback'
  		# add new waypoint to self._waypoint array or waypoints
		self._waypoints[self._num_waypoints].w0      = msg.w[0]
		self._waypoints[self._num_waypoints].w1      = msg.w[1]
		self._waypoints[self._num_waypoints].w2      = msg.w[2]
		self._waypoints[self._num_waypoints].chi_d     = msg.chi_d
		self._waypoints[self._num_waypoints].chi_valid = msg.chi_valid
		self._waypoints[self._num_waypoints].Va_d      = msg.Va_d
		self._num_waypoints+=1

	def current_path_publisher(self, output):
		print 'Current Path Publisher'
		current_path = FW_Current_Path()
		# set current_path to output from manager
		current_path.flag = output.flag
		current_path.Va_d = output.Va_d
		for i in range(0,3):
			current_path.r[i] = output.r[i]
			current_path.q[i] = output.q[i]
			current_path.c[i] = output.c[i]
		current_path.rho = output.rho
		current_path.lambda_ = output.lambda_

		self._current_path_pub.publish(current_path) # publish

	# Classes in class
	class dubinspath_s:
		ps = np.array([0.0, 0.0, 0.0])
		chis = 0.0
		pe = np.array([0.0, 0.0, 0.0])
		chie = 0.0
		R = 0.0
		L = 0.0
		cs = np.array([0.0, 0.0, 0.0])
		lams = 0
		ce = np.array([0.0, 0.0, 0.0])
		lame = 0
		w1 = np.array([0.0, 0.0, 0.0])
		q1 = np.array([0.0, 0.0, 0.0])
		w2 = np.array([0.0, 0.0, 0.0])
		w3 = np.array([0.0, 0.0, 0.0])
		q3 = np.array([0.0, 0.0, 0.0])

	# Member objects
	_dubinspath = dubinspath_s()
	index_a = 0	

	# functions
	def manage(self, params, inpt, output):
		print 'Manage'
		if (self._num_waypoints < 2):
			output.flag = True
			output.Va_d = 9
			output.r[0] = inpt.pn
			output.r[1] = inpt.pe
			output.r[2] = -inpt.h
			output.q[0] = cos(inpt.chi)
			output.q[1] = sin(inpt.chi)
			output.q[2] = 0.0
			output.c[0] = 0.0
			output.c[1] = 0.0
			output.c[2] = 0.0
			output.rho = 0
			output.lambda_ = 0
		else:

			if (self._waypoints[self.index_a].chi_valid): 
				print 'Manage -- Dubins'
				output = self.manage_dubins(params, inpt, output)
			else:
				# print 'Manage -- Line'
				# output = self.manage_line(params, inpt, output)
				print 'Manage -- Fillet'
				self.manage_fillet(params,inpt,output)
		return output

	def manage_line(self, params, inpt, output):
		print 'Def Manage Line'
		# print params.R_min
		p = np.array([inpt.pn, inpt.pe, -inpt.h])

		a = self._waypoints[self.index_a]
		b = self.waypoint_temp()
		c = self.waypoint_temp()

		if (self.index_a == (self._num_waypoints - 1)):
			b = self._waypoints[0]
			c = self._waypoints[1]
		elif (self.index_a == (self._num_waypoints - 2)):
			b = self._waypoints[self._num_waypoints - 1]
			c = self._waypoints[0]
		else:
			b = self._waypoints[self.index_a + 1]
			c = self._waypoints[self.index_a + 2]

		# print 'waypoint b'
		# print b
		# print 'waypoint c'
		# print c

		w_im1 = np.array([a.w0,a.w1,a.w2])
		w_i = np.array([b.w0,b.w1,b.w2])
		w_ip1 = np.array([c.w0,c.w1,c.w2])

		output.flag = True
		output.Va_d = a.Va_d
		output.r = [w_im1[0],w_im1[1],w_im1[2]]

		q_im1 = self.normalize(w_i - w_im1)
		q_i = self.normalize(w_ip1 - w_i)
		output.q = [q_im1[0],q_im1[1],q_im1[2]]
		output.c = [1, 1, 1]
		output.rho = 1
		output.lambda_ = 1

		n_i = self.normalize(q_im1 + q_i)
		if (self.dot((p - w_i),n_i) > 0.0):
			if (self.index_a == (_num_waypoints - 1)):
				self.index_a = 0
			else:
				self.index_a += 1

		return output

	def manage_fillet(self, params, inpt, output):
		print 'Def Manage Fillet'

		p = np.array([inpt.pn, inpt.pe, -inpt.h])

		a = self._waypoints[self.index_a]
		b = self.waypoint_temp()
		c = self.waypoint_temp()

		if (self.index_a == (self._num_waypoints - 1)):
			b = self._waypoints[0]
			c = self._waypoints[1]
		elif (self.index_a == (self._num_waypoints - 2)):
			b = self._waypoints[self._num_waypoints - 1]
			c = self._waypoints[0]
		else:
			b = self._waypoints[self.index_a + 1]
			c = self._waypoints[self.index_a + 2]

		# print 'waypoint b'
		# print b
		# print 'waypoint c'
		# print c

		w_im1 = np.array([a.w0,a.w1,a.w2])
		w_i = np.array([b.w0,b.w1,b.w2])
		w_ip1 = np.array([c.w0,c.w1,c.w2])

		R_min = params.R_min

		output.Va_d = a.Va_d
		output.r = [w_im1[0],w_im1[1],w_im1[2]]

		q_im1 = self.normalize(w_i - w_im1)
		q_i = self.normalize(w_ip1 - w_i)
		beta = acos(self.dot(-q_im1, q_i))

		z = np.array([0.0, 0.0, 0.0])

		if self.fillet_state == 'Straight':
			output.flag = True
			output.q = [q_im1[0], q_im1[1], q_im1[2]]
			output.c = [1, 1, 1]
			output.rho = 1
			output.lambda_ = 1
			z = w_i - q_im1*(R_min/tan(beta/2))
			if(self.dot((p-z),q_im1) > 0):
			    self.fillet_state = 'Orbit'
		elif self.fillet_state == 'Obrbit':
			output.flag = True
			output.q = [q_i1[0], q_i1[1], q_i1[2]]
			c = w_i - self.normalize(q_im1-q_i)*(R_min/sin(beta/2.0))
			output.c = [c[0], c[1], c[2]]
			output.rho = R_min
			output.lambda_ = 1 if (q_im1[0]*q_i[1]-q_im1[1]*q_i[0]) > 0 else -1
			if (self.dot((p-z),q_i) > 0):
				if self.index_a == (self._num_waypoints - 1):
					self.index_a = 0
				else:
					self.index_a += 1
				self.fillet_state = 'Straight'


	def manage_dubins(self, params, inpt, output):
		print 'Def Manage Dubins'

		p = np.array([inpt.pn, inpt.pe, -inpt.h])

		R_min = params.R_min

		if self.dubin_state == 'First':
			self.dubinsParameters(self._waypoints[0], self._waypoints[1], R_min)
			output.flag = False
			output.Va_d = self._waypoints[self.index_a].Va_d;
			output.r[0] = 0
			output.r[1] = 0
			output.r[2] = 0
			output.q[0] = 0
			output.q[1] = 0
			output.q[2] = 0
			output.c[0] = self._dubinspath.cs[0]
			output.c[1] = self._dubinspath.cs[1]
			output.c[2] = self._dubinspath.cs[2]
			output.rho = self._dubinspath.R
			output.lambda_ = self._dubinspath.lams
			if(self.dot((p - self._dubinspath.w1), self._dubinspath.q1) >= 0): # start in H1
				self.dubin_state ='Before_H1_wrong_side'
			else:
				self.dubin_state = 'Before_H1'

		elif self.dubin_state == 'Before_H1':
			output.flag = False
			output.Va_d = self._waypoints[self.index_a].Va_d
			output.r[0] = 0
			output.r[1] = 0
			output.r[2] = 0
			output.q[0] = 0
			output.q[1] = 0
			output.q[2] = 0
			output.c[0] = self._dubinspath.cs[0]
			output.c[1] = self._dubinspath.cs[1]
			output.c[2] = self._dubinspath.cs[2]
			output.rho = self._dubinspath.R
			output.lambda_ = self._dubinspath.lams
			if(self.dot(p - self._dubinspath.w1, self._dubinspath.q1) >= 0): # entering H1
				self.dubin_state ='Straight'

		elif self.dubin_state == 'Before_H1_wrong_side':
			output.flag = False
			output.Va_d = self._waypoints[self.index_a].Va_d
			output.r[0] = 0
			output.r[1] = 0
			output.r[2] = 0
			output.q[0] = 0
			output.q[1] = 0
			output.q[2] = 0
			output.c[0] = self._dubinspath.cs[0]
			output.c[1] = self._dubinspath.cs[1]
			output.c[2] = self._dubinspath.cs[2]
			output.rho = self._dubinspath.R
			output.lambda_ = _dubinspath.lams
			if(self.dot(p - self._dubinspath.w1, self._dubinspath.q1) < 0): # exit H1
				self.dubin_state ='Before_H1'

		elif self.dubin_state == 'Straight':
			output.flag = True
			output.Va_d = self._waypoints[self.index_a].Va_d
			output.r[0] = self._dubinspath.w1[0]
			output.r[1] = self._dubinspath.w1[1]
			output.r[2] = self._dubinspath.w1[2]
			output.q[0] = self._dubinspath.q1[0]
			output.q[1] = self._dubinspath.q1[1]
			output.q[2] = self._dubinspath.q1[2]
			output.c[0] = 1
			output.c[1] = 1
			output.c[2] = 1
			output.rho = 1
			output.lambda_ = 1
			if(self.dot(p - self._dubinspath.w2, self._dubinspath.q1) >= 0): # entering H2
				if(self.dot(p - self._dubinspath.w3, self._dubinspath.q3) >= 0): # start in H3
					dubin_state = 'Before_H3_wrong_side'
				else:
					dubin_state = 'Before_H3'

		elif self.dubin_state == 'Before_H3':
			output.flag = False
			output.Va_d = self._waypoints[self.index_a].Va_d
			output.r[0] = 0
			output.r[1] = 0
			output.r[2] = 0
			output.q[0] = 0
			output.q[1] = 0
			output.q[2] = 0
			output.c[0] = self._dubinspath.ce[0]
			output.c[1] = self._dubinspath.ce[1]
			output.c[2] = self._dubinspath.ce[2]
			output.rho = self._dubinspath.R
			output.lambda_ = self._dubinspath.lame
			if(self.dot(p - self._dubinspath.w3, self._dubinspath.q3) >= 0): # entering H3
				# increase the waypoint pointer
				b = self.waypoint_temp()

				if(self.index_a == (self._num_waypoints-1)):
					self.index_a = 0
					a = self._waypoints[self.index_a]
					b = self._waypoints[1]
				elif(self.index_a == (self._num_waypoints-2)):
					self.index_a += 1
					a = self._waypoints[self.index_a]
					b = self._waypoints[0]
				else:
					self.index_a += 1
					a = self._waypoints[self.index_a]
					b = self._waypoints[self.index_a + 1]
					
				# plan new Dubin's path to next waypoint configuration
				self.dubinsParameters(a, b, R_min)

				#start new path
				if(self.dot(p - self._dubinspath.w1, self._dubinspath.q1) >= 0): # start in H1
					dubin_state = 'Before_H1_wrong_side'
				else:
					dubin_state = 'Before_H1'
				

		elif self.dubin_state == 'Before_H3_wrong_side':
			output.flag = False
			output.Va_d = self._waypoints[self.index_a].Va_d
			output.r[0] = 0
			output.r[1] = 0
			output.r[2] = 0
			output.q[0] = 0
			output.q[1] = 0
			output.q[2] = 0
			output.c[0] = self._dubinspath.ce[0]
			output.c[1] = self._dubinspath.ce[1]
			output.c[2] = self._dubinspath.ce[2]
			output.rho = self._dubinspath.R
			output.lambda_ = self._dubinspath.lame
			if(self.dot(p - self._dubinspath.w3, self._dubinspath.q3) < 0): # exit H3
				dubin_state = 'Before_H1'

		return output

	def rotz(self, theta):
		R = np.array([[cos(theta), -sin(theta), 0.0],
						[sin(theta),cos(theta),0.0],
						[0.0, 0.0, 1.0]])
		return R

	def mo(self,inpt):
		val = 0.0
		if (inpt > 0):
			val = fmod(inpt, 2*M_PI_F)
		else:
			n = 0.0
			n = floor(inpt/2/M_PI_F)
			val = inpt - n*2*M_PI_F
		return val

	def normalize(self, v):
		norm=np.linalg.norm(v, ord=1)
		if norm==0:
			norm=np.finfo(v.dtype).eps
		return v/norm

	def dubinsParameters(self, start_node, end_node, R):
		ell = sqrt((start_node.w0 - end_node.w0)*(start_node.w0 - end_node.w0) + (start_node.w1 - end_node.w1)*(start_node.w1 - end_node.w1))
		if (ell < 2*R):
			print 'The distance between nodes must be larger than 2R'
		else:
			self._dubinspath.ps[0] = start_node.w0
			self._dubinspath.ps[1] = start_node.w1
			self._dubinspath.ps[2] = start_node.w2
			self._dubinspath.chis = start_node.chi_d
			self._dubinspath.pe[0] = end_node.w0
			self._dubinspath.pe[1] = end_node.w1
			self._dubinspath.pe[2] = end_node.w2
			self._dubinspath.chie = end_node.chi_d

			crs = self._dubinspath.ps
			crs[0] += R*(cos(M_PI_2_F)*cos(self._dubinspath.chis) - sin(M_PI_2_F)*sin(self._dubinspath.chis))
			crs[1] += R*(sin(M_PI_2_F)*cos(self._dubinspath.chis) + cos(M_PI_2_F)*sin(self._dubinspath.chis))
			clss = self._dubinspath.ps
			clss[0] += R*(cos(-M_PI_2_F)*cos(self._dubinspath.chis) - sin(-M_PI_2_F)*sin(self._dubinspath.chis))
			clss[1] += R*(sin(-M_PI_2_F)*cos(self._dubinspath.chis) + cos(-M_PI_2_F)*sin(self._dubinspath.chis))
			cre = self._dubinspath.pe
			cre[0] += R*(cos(M_PI_2_F)*cos(self._dubinspath.chie) - sin(M_PI_2_F)*sin(self._dubinspath.chie))
			cre[1] += R*(sin(M_PI_2_F)*cos(self._dubinspath.chie) + cos(M_PI_2_F)*sin(self._dubinspath.chie))
			cle = self._dubinspath.pe
			cle[0] += R*(cos(-M_PI_2_F)*cos(self._dubinspath.chie) - sin(-M_PI_2_F)*sin(self._dubinspath.chie))
			cle[1] += R*(sin(-M_PI_2_F)*cos(self._dubinspath.chie) + cos(-M_PI_2_F)*sin(self._dubinspath.chie))

			# compute L1
			theta = atan2(cre[1] - crs[1], cre[0] - crs[0])

			L1 = len(crs - cre) + R*self.mo(2*M_PI_F + self.mo(theta - M_PI_2_F) - self.mo(self._dubinspath.chis - M_PI_2_F)) + \
                R*self.mo(2*M_PI_F + self.mo(self._dubinspath.chie - M_PI_2_F) - self.mo(theta - M_PI_2_F))

            # compute L2
			ell = len(cle - crs)
			theta = atan2(cle[1] - crs[1], cle[0] - crs[0])
			if(2*R/ell > 1.0 or 2*R/ell < -1.0):
				L2 = 9999.0
			else:
				theta2 = theta - M_PI_2_F + asin(2*R/ell)
				L2 = sqrt(ell*ell - 4*R*R) + R*self.mo(2*M_PI_F + self.mo(theta2) - self.mo(self._dubinspath.chis - M_PI_2_F)) + \
				   R*self.mo(2*M_PI_F + self.mo(theta2 + M_PI_F) - self.mo(self._dubinspath.chie + M_PI_2_F))

		    # compute L3
			ell = len(cre - clss)
			theta = atan2(cre[1] - clss[1], cre[0] - clss[0])
			if (2*R/ell > 1.0 or 2*R/ell < -1.0):
				L3 = 9999.0
			else:
				theta2 = acos(2*R/ell)
				L3 = sqrt(ell*ell - 4*R*R) + R*self.mo(2*M_PI_F + self.mo(self._dubinspath.chis + M_PI_2_F) - self.mo(theta + theta2)) + \
			         R*self.mo(2*M_PI_F + self.mo(self._dubinspath.chie - M_PI_2_F) - self.mo(theta + theta2 - M_PI_F))

			# compute L4
			theta = atan2(cle[1]-clss[1],cle[0]-clss[0])
			L4 = len(clss - cle) + R*self.mo(2*M_PI_F + self.mo(self._dubinspath.chis + M_PI_2_F) - self.mo(theta + M_PI_2_F)) + \
			     R*self.mo(2*M_PI_F + self.mo(theta + M_PI_2_F) - self.mo(self._dubinspath.chie + M_PI_2_F))

			# L = minimum distance
			idx = 1
			self._dubinspath.L = L1
			if (L2 < self._dubinspath.L):
				self._dubinspath.L = L2
				idx = 2
			if (L3 < self._dubinspath.L):
				self._dubinspath.L = L3
				idx = 3
			if (L4 < self._dubinspath.L):
				self._dubinspath.L = L4
				idx = 4

			e1 = np.array([1.0, 0.0, 0.0])

			# cpp = switch statement here
			if idx == 1:
				self._dubinspath.cs = crs
				self._dubinspath.lams = 1
				self._dubinspath.ce = cre
				self._dubinspath.lame = 1
				self._dubinspath.q1 = self.normalize(cre - crs)
				self._dubinspath.w1 = self._dubinspath.cs + np.dot(self.rotz(-M_PI_2_F),self._dubinspath.q1)*R
				self._dubinspath.w2 = self._dubinspath.ce + np.dot(self.rotz(-M_PI_2_F),self._dubinspath.q1)*R
			elif idx == 2:
				self._dubinspath.cs = crs
				self._dubinspath.lams = 1
				self._dubinspath.ce = cle
				self._dubinspath.lame = -1
				ell = len(cle - crs)
				theta = atan2(cle(1) - crs(1), cle(0) - crs(0))
				theta2 = theta - M_PI_2_F + asin(2*R/ell)
				self._dubinspath.q1 = np.dot(self.rotz(theta2 + M_PI_2_F),e1)
				self._dubinspath.w1 = self._dubinspath.cs + np.dot(self.rotz(theta2),e1)*R
				self._dubinspath.w2 = self._dubinspath.ce + np.dot(self.rotz(theta2 + M_PI_F),e1)*R
			elif idx == 3:
				self._dubinspath.cs = clss
				self._dubinspath.lams = -1
				self._dubinspath.ce = cre
				self._dubinspath.lame = 1
				ell = len(cre - clss)
				theta = atan2(cre(1) - clss(1), cre(0) - clss(0))
				theta2 = acos(2*R/ell)
				self._dubinspath.q1 = np.dot(self.rotz(theta + theta2 - M_PI_2_F),e1)
				self._dubinspath.w1 = self._dubinspath.cs + np.dot(self.rotz(theta + theta2),e1)*R
				self._dubinspath.w2 = self._dubinspath.ce + np.dot(self.rotz(theta + theta2 - M_PI_F),e1)*R
			elif idx == 4:
				self._dubinspath.cs = clss
				self._dubinspath.lams = -1
				self._dubinspath.ce = cle
				self._dubinspath.lame = -1
				self._dubinspath.q1 = self.normalize(cle - clss)
				self._dubinspath.w1 = self._dubinspath.cs + np.dot(self.rotz(M_PI_2_F),self._dubinspath.q1)*R
				self._dubinspath.w2 = self._dubinspath.ce + np.dot(self.rotz(M_PI_2_F),self._dubinspath.q1)*R
			self._dubinspath.w3 = self._dubinspath.pe
			self._dubinspath.q3 = np.dot(self.rotz(self._dubinspath.chie),e1)
			self._dubinspath.R = R

	def dot(self, first, second):
		# first and second are np.arrays of size 3
		return first[0]*second[0] + first[1]*second[1] + first[2]*second[2]

	def waypointprint(self):
		for _ in range(0,self._num_waypoints):
			print 'waypoint#' + str(_)
			print self._waypoints[_].w0
			print self._waypoints[_].w1
			print self._waypoints[_].w2
			print self._waypoints[_].chi_d
			print self._waypoints[_].chi_valid
			print self._waypoints[_].Va_d

##############################
#### Main Function to Run ####
##############################
if __name__ == '__main__':
	# Initialize Node
	rospy.init_node('ros_plane_path_manager')

	# set rate
	hz = 10.0
	rate = rospy.Rate(hz)

	# init path_manager_base object
	manager = path_manager_base()

	# Loop
	while not rospy.is_shutdown():
		rate.sleep()