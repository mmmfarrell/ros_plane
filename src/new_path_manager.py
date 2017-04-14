#!/usr/bin/env python
# Python implementation of "path_manager"
# path_manager_base.h & path_manager_base.py
# need to make polymorphism

import rospy
from fcu_common.msg import FW_State, FW_Current_Path, FW_Waypoint
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import Float32, Float32MultiArray
from math import *
import numpy as np
#import Eigen
#from ros_plane import ControllerConfig

# These were global constants in the cpp files, can change to np.pi and np.pi/2 
M_PI_F = 3.14159265358979323846
M_PI_2_F = 1.57079632679489661923

def crossed_plane(plane_point, normal, loc):
	crossed = False
	point_diff = loc - plane_point
	# print 'loc'
	# print loc
	# print 'plane_point'
	# print plane_point
	# print 'normal'
	# print normal
	dot_prod = np.dot(point_diff.reshape((1, 3)), normal)
	if dot_prod > 0:
		crossed = True
	return crossed
	
def mod2pi(phi):
	mod = phi%(2 * np.pi)
	return mod

class path_manager_base:

	# Init function
	def __init__(self):
		# print 'Base Init'

		# Init Params
		self.params = self.params_s()
		self.params.R_min = rospy.get_param('R_min', 200.0)

		# inititlialize subscribers
		self._vehicle_state_sub = rospy.Subscriber('state', FW_State, self.vehicle_state_callback)
		self._new_waypoint_sub = rospy.Subscriber('waypoint_path', FW_Waypoint, self.new_waypoint_callback)

		# Init Publishers
		self._current_path_pub = rospy.Publisher('current_path', FW_Current_Path, queue_size=10)
		self._current_wp_pub = rospy.Publisher('current_waypoint', FW_Waypoint, queue_size=10)

		self.index_a = 1 #waypoint 0 should be initial position, waypoint 1 where you want to start flying to	

		# Class members
		self._num_waypoints = 0
		self._vehicle_state = FW_State()

		self._dubins_mark = self.dubin_state()
		self._dub_state = self._dubins_mark.First #self.dubin_state.First # = 'First' #'First', 'Before_H1', 'Before_H1_wrong_side', 'Straight', 'Before_H3', 'Before_H3_wrong_side'
		self.state = 1
		# Member objects
		self._dubinspath = self.dubinspath_s()

		self._waypoints = []

		# run waypoint init to initialize with waypoints found in waypoint_init (alternative to path_planner.py)
		self.waypoint_init()

	# Subclasses
	class waypoint_s:
		w = [0.0, 0.0, 0.0] # position NED
		chi_d = 0.0 # heading at waypoint
		chi_valid = True # False = don't worry about heading at waypoint
		Va_d = 0.0 # airspeed at waypoint

		def __str__(self):
			return "w: " + str(self.w) + "\nchi_d: " + str(self.chi_d) + "\nchi_valid: " + str(self. chi_valid) + "\nVa_d: " + str(self.Va_d)
		def __repr__(self):
			return str(self)
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

	class dubin_state: 
		First = 0
		Before_H1 = 1
		Before_H1_wrong_side = 2
		Straight = 3
		Before_H3 = 4
		Before_H3_wrong_side = 5


	# Class Member Functions
	def waypoint_init(self):
		# print 'Waypoint Init'

		new_wp = self.waypoint_temp()
		self._waypoints.append(new_wp)
		self._waypoints[self._num_waypoints].w0  	 = 0
		self._waypoints[self._num_waypoints].w1      = 0
		self._waypoints[self._num_waypoints].w2      = -100
		self._waypoints[self._num_waypoints].chi_d     = 0.0#-9992
		self._waypoints[self._num_waypoints].chi_valid = True
		self._waypoints[self._num_waypoints].Va_d      = 35
		self._num_waypoints+=1

		new_wp = self.waypoint_temp()
		self._waypoints.append(new_wp)
		self._waypoints[self._num_waypoints].w0      = 1000
		self._waypoints[self._num_waypoints].w1      = 0
		self._waypoints[self._num_waypoints].w2      = -100
		self._waypoints[self._num_waypoints].chi_d     = 0.0#-9993
		self._waypoints[self._num_waypoints].chi_valid = True
		self._waypoints[self._num_waypoints].Va_d      = 35
		self._num_waypoints+=1

		new_wp = self.waypoint_temp()
		self._waypoints.append(new_wp)
		self._waypoints[self._num_waypoints].w0      = 2000
		self._waypoints[self._num_waypoints].w1      = 0
		self._waypoints[self._num_waypoints].w2      = -100
		self._waypoints[self._num_waypoints].chi_d     = -0.5#-9994
		self._waypoints[self._num_waypoints].chi_valid = True
		self._waypoints[self._num_waypoints].Va_d      = 35
		self._num_waypoints+=1

		new_wp = self.waypoint_temp()
		self._waypoints.append(new_wp)
		self._waypoints[self._num_waypoints].w0      = 2500
		self._waypoints[self._num_waypoints].w1      = 500
		self._waypoints[self._num_waypoints].w2      = -100
		self._waypoints[self._num_waypoints].chi_d     = 0.0#-9995
		self._waypoints[self._num_waypoints].chi_valid = True
		self._waypoints[self._num_waypoints].Va_d      = 35
		self._num_waypoints+=1

		new_wp = self.waypoint_temp()
		self._waypoints.append(new_wp)
		self._waypoints[self._num_waypoints].w0      = 3000
		self._waypoints[self._num_waypoints].w1      = 750
		self._waypoints[self._num_waypoints].w2      = -100
		self._waypoints[self._num_waypoints].chi_d     = 0.75#-9996
		self._waypoints[self._num_waypoints].chi_valid = True
		self._waypoints[self._num_waypoints].Va_d      = 35
		self._num_waypoints+=1

		print('Waypoints inititlialized: ')
		print 'Number of Waypoints: ' + str(len(self._waypoints))

	def vehicle_state_callback(self, msg):
		# print 'Vehicle State Callback'
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
		# print 'New Waypoint Callback'
		# add new waypoint to self._waypoint array or waypoints
		self._waypoints[self._num_waypoints].w0      = msg.w[0]
		self._waypoints[self._num_waypoints].w1      = msg.w[1]
		self._waypoints[self._num_waypoints].w2      = msg.w[2]
		self._waypoints[self._num_waypoints].chi_d     = msg.chi_d
		self._waypoints[self._num_waypoints].chi_valid = msg.chi_valid
		self._waypoints[self._num_waypoints].Va_d      = msg.Va_d
		self._num_waypoints+=1

	def current_path_publisher(self, output):
		# print 'Current Path Publisher'
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

		current_wp = FW_Waypoint()
		current_wp.w[0] = self._waypoints[self.index_a].w0
		current_wp.w[1] = self._waypoints[self.index_a].w1
		current_wp.w[2] = self._waypoints[self.index_a].w2
		current_wp.chi_d = self._waypoints[self.index_a].chi_d
		current_wp.chi_valid = self._waypoints[self.index_a].chi_valid
		current_wp.Va_d= self._waypoints[self.index_a].Va_d
		self._current_wp_pub.publish(current_wp) # publish waypoint header toward

	# Classes in class
	class dubinspath_s:
		ps = np.array([0.0, 0.0, 0.0]) 	# start position
		chis = 0.0						# start course angle
		pe = np.array([0.0, 0.0, 0.0])	# End Position
		chie = 0.0						# end course angle
		R = 0.0							# turn radius
		L = 0.0							# length of path
		cs = np.array([0.0, 0.0, 0.0])	# center of the start circle
		lams = 0						# direction of the start circle
		ce = np.array([0.0, 0.0, 0.0])	# center of the end circle
		lame = 0						# direction of the end circle
		w1 = np.array([0.0, 0.0, 0.0])	# vector defining half plane H1
		q1 = np.array([0.0, 0.0, 0.0])	# unit vector along straight line path
		w2 = np.array([0.0, 0.0, 0.0]) 	# vector defining half plane H2
		w3 = np.array([0.0, 0.0, 0.0])	# vector defining half plane H3
		q3 = np.array([0.0, 0.0, 0.0])	# unit vector defining direction of half plane H3
	

	# functions
	def manage(self, params, inpt, output):
		# print 'Manage'
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
			rospy.logwarn('ERROR: less than 2 waypoints!!!')
		else:
			# print self.index_a
			if (self._waypoints[self.index_a].chi_valid) and (self.index_a > 1): 
				# print 'Manage -- Dubins'
				output = self.manage_dubins(params, inpt, output)
			elif (self._waypoints[self.index_a].chi_valid):
				# print 'Manage -- Line'
				output = self.manage_line(params, inpt, output)
			else:
				# print 'Manage -- Line'
				# output = self.manage_line(params, inpt, output)
				# print 'Manage -- Fillet'
				# self.manage_fillet(params,inpt,output)
				rospy.logwarn('ERROR: MUST RUN DUBINS PATH (set chi_valid to True)')
		return output

	


	def manage_dubins(self, params, inpt, output):
		# print 'Def Manage Dubins'

		pos = np.array([inpt.pn, inpt.pe, -inpt.h])

		R_min = params.R_min

		assert (self._num_waypoints >= 3), "insufficient waypoints"

		now = self._waypoints[self.index_a]
		past = self.waypoint_temp()

		if (self.index_a == 0):
			past = self._waypoints[self._num_waypoints]
		else:
			past = self._waypoints[self.index_a - 1]

		w_past = np.array([past.w0, past.w1, past.w2])
		chi_past = past.chi_d

		w_now = np.array([now.w0, now.w1, now.w2])
		chi_now = now.chi_d

		[L, cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3] = self.dubinsParameters(w_past, chi_past, w_now, chi_now, R_min)
		r = 'r'
		q = 'q'
		c = 'c'
		lam = 'lam'
		flag = 'flag'
		if self.state == 1:
			flag = False
			c = cs
			lam = lam_s
			if crossed_plane(z1, -q1, pos):
				self.state = 2
		if self.state == 2:
			flag = False
			c = cs
			lam = lam_s
			if crossed_plane(z1, q1, pos):
				self.state = 3
		if self.state == 3:
			flag = True
			r = z1
			q = q1
			if crossed_plane(z2, q1, pos):
				self.state = 4
		if self.state == 4:
			flag = False
			c = ce
			lam = lam_e
			if crossed_plane(z3, -q3, pos):
				self.state = 5
		if self.state == 5:
			flag = False
			c = ce
			lam = lam_e
			if crossed_plane(z3, q3, pos):
				self.state = 1
				self.waypoint_counter += 1

		# return [flag, self.vg_des, r, q, c, self.t_rad, lam]
		output.flag = flag # Inicates strait line or orbital path (true is line, false is orbit)
		output.Va_d = now.Va_d # Desired airspeed (m/s)
		output.r = r # Vector to origin of straight line path (m)
		output.q = q # Unit vector, desired direction of travel for line path
		output.c = c # Center of orbital path (m)
		output.rho = R_min # Radius of orbital path (m)
		output.lambda_ = lam # Direction of orbital path (cw is 1, ccw is -1)

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
		# Theres probably a better way to do this, but his is what works
		# norm=np.linalg.norm(v, ord=1) # tried this, but it didn't work, result in vector that sums to 1 not magnitude 1
		norm = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
		if norm==0:
			norm=np.finfo(v.dtype).eps
		temp = np.array([0.0, 0.0, 0.0])
		temp[0] = v[0]/norm
		temp[1] = v[1]/norm
		temp[2] = v[2]/norm
		return temp#v/norm

	def dubinsParameters(self, ps, chi_s, pe, chi_e, R):
		# ps = np.array([start_node.w0, start_node.w1, start_node.w2])
		# chi_s = start_node.chi_d
		# pe = np.array([end_node.w0, end_node.w1, end_node.w2])
		# chi_e = end_node.chi_d

		dist = np.linalg.norm(ps - pe)
		e1 = np.array([[1], [0], [0]])

		assert (dist >= 3 * R), "waypoints are too close together!"
		# assert (self.fillet_rad > minTurnRad)
		crs = ps + R * np.dot(self.rotz(np.pi/2), np.array([[cos(chi_s)], [sin(chi_s)], [0]]))
		cls = ps + R * np.dot(self.rotz(-np.pi/2), np.array([[cos(chi_s)], [sin(chi_s)], [0]]))
		cre = pe + R * np.dot(self.rotz(np.pi/2), np.array([[cos(chi_e)], [sin(chi_e)], [0]]))
		cle = pe + R * np.dot(self.rotz(-np.pi/2), np.array([[cos(chi_e)], [sin(chi_e)], [0]]))

		#compute length for case 1 rsr
		ang = atan2(cre.item(1)-crs.item(1), cre.item(0)-crs.item(0))
		L1 = np.linalg.norm(crs-cre) + R * mod2pi(2 * np.pi + mod2pi(ang - np.pi / 2) - mod2pi(chi_s - np.pi / 2)) \
			 + R * mod2pi(2 * np.pi + mod2pi(chi_e - np.pi / 2) - mod2pi(ang - np.pi / 2))

		# Compute length for case 2 rsl
		ang = atan2(cle.item(1)-crs.item(1), cle.item(0)-crs.item(0))
		l = np.linalg.norm(cle - crs)
		ang2 = ang - np.pi / 2 + asin((2 * R) / l)
		L2 = np.sqrt(l ** 2 - 4 * R ** 2) + R * mod2pi(2 * np.pi + mod2pi(ang2) - mod2pi(chi_s - np.pi / 2)) \
			 + R * mod2pi(2 * np.pi + mod2pi(ang2 + np.pi) - mod2pi(chi_e + np.pi / 2))

		# Compute length for case 3 lsr
		ang = atan2(cre.item(1)-cls.item(1), cre.item(0)-cls.item(0))
		l = np.linalg.norm(cre-cls)
		ang2 = acos((2 * R) / l)
		L3 = np.sqrt(l ** 2 - 4 * R ** 2) + R * mod2pi(2 * np.pi + mod2pi(chi_s + np.pi / 2) - mod2pi(ang + ang2)) \
			 + R * mod2pi(2 * np.pi + mod2pi(chi_e - np.pi / 2) - mod2pi(ang + ang2 - np.pi))

		# Compute length for case 4 lsl
		ang = atan2(cle.item(1)-cls.item(1), cle.item(0)-cls.item(0))
		L4 = np.linalg.norm(cls-cle) + R * mod2pi(2 * np.pi + mod2pi(chi_s + np.pi / 2) - mod2pi(ang + np.pi / 2)) \
			 + R * mod2pi(2 * np.pi + mod2pi(ang + np.pi / 2) - mod2pi(chi_e + np.pi / 2))

		lengths = [L1, L2, L3, L4]
		if min(lengths) == L1:
			cs = crs
			lam_s = 1
			ce = cre
			lam_e = 1
			q1 = (ce - cs) / np.linalg.norm(ce - cs)
			z1 = cs + R * np.dot(self.rotz(-np.pi/2), q1)
			z2 = ce + R * np.dot(self.rotz(-np.pi/2), q1)

		elif min(lengths) == L2:
			cs = crs
			lam_s = 1
			ce = cle
			lam_e = -1
			l = np.linalg.norm(ce - cs)
			ang = atan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
			ang2 = ang - np.pi/2 + asin((2 * R) / l)
			q1 = np.dot(self.rotz(ang2 + np.pi/2), e1)
			z1 = cs + R * np.dot(self.rotz(ang2), e1)
			z2 = ce + R * np.dot(self.rotz(ang2 + np.pi), e1)

		elif min(lengths) == L3:
			cs = cls
			lam_s = -1
			ce = cre
			lam_e = 1
			l = np.linalg.norm(ce - cs)
			ang = atan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
			ang2 = acos((2 * R) / l)
			q1 = np.dot(self.rotz(ang + ang2 - np.pi/2), e1)
			z1 = cs + R * np.dot(self.rotz(ang + ang2), e1)
			z2 = ce + R * np.dot(self.rotz(ang + ang2 -np.pi), e1)

		# elif min(lengths) == L4:
		else:
			cs = cls
			lam_s = -1
			ce = cle
			lam_e = -1
			q1 = (ce - cs) / np.linalg.norm(ce - cs)
			z1 = cs + R * np.dot(self.rotz(np.pi/2), q1)
			z2 = ce + R * np.dot(self.rotz(np.pi/2), q1)

		z3 = pe
		q3 = np.dot(self.rotz(chi_e), e1)

		print 'cs'
		print cs
		print 'lam_s'
		print lam_s
		print 'ce'
		print lam_e
		print 'z1'
		print z1
		print 'q1'
		print q1
		print 'z2'
		print z2
		print 'z3'
		print z3
		print 'q3'
		print q3

		self._dubinspath.ps = ps 	# start position
		self._dubinspath.chis = chi_s						# start course angle
		self._dubinspath.pe = pe	# End Position
		self._dubinspath.chie = chi_e						# end course angle
		self._dubinspath.R = R							# turn radius
		self._dubinspath.L = min(lengths)							# length of path
		self._dubinspath.cs = cs	# center of the start circle
		self._dubinspath.lams = lam_s						# direction of the start circle
		self._dubinspath.ce = ce	# center of the end circle
		self._dubinspath.lame = lam_e						# direction of the end circle
		self._dubinspath.w1 = z1	# vector defining half plane H1
		self._dubinspath.q1 = q1	# unit vector along straight line path
		self._dubinspath.w2 = z2 	# vector defining half plane H2
		self._dubinspath.w3 = z3	# vector defining half plane H3
		self._dubinspath.q3 = q3	# unit vector defining direction of half plane H3
		rospy.logwarn('INFO: Dubins params updated')

		return [min(lengths), cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3]

	def dot(self, first, second):
		# first and second are np.arrays of size 3
		temp = (first[0]*second[0] + first[1]*second[1] + first[2]*second[2])
		return temp

	def waypointprint(self):
		for _ in range(0,self._num_waypoints):
			print 'waypoint#' + str(_)
			print self._waypoints[_].w0
			print self._waypoints[_].w1
			print self._waypoints[_].w2
			print self._waypoints[_].chi_d
			print self._waypoints[_].chi_valid
			print self._waypoints[_].Va_d

	def manage_line(self, params, inpt, output):
		print 'Def Manage Line'
		# print params.R_min
		p = np.array([inpt.pn, inpt.pe, -inpt.h])

		b = self._waypoints[self.index_a]
		a = self.waypoint_temp()
		c = self.waypoint_temp()

		if (self.index_a == (self._num_waypoints)):
			a = self._waypoints[self.index_a-1]
			c = self._waypoints[0]
		elif (self.index_a == 0):
			a = self._waypoints[self._num_waypoints]
			c = self._waypoints[self.index_a + 1]
		else:
			a = self._waypoints[self.index_a - 1]
			c = self._waypoints[self.index_a + 1]
		# print 'waypoint a'
		# print a
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
			if (self.index_a == (self._num_waypoints - 1)):
				self.index_a = 0
			else:
				self.index_a += 1

		return output

	def crossed_plane(plane_point, normal, loc):
		crossed = False
		point_diff = loc - plane_point
		dot_prod = np.dot(point_diff.reshape((1, 3)), normal)
		if dot_prod > 0:
			crossed = True
		return crossed

##############################
#### Main Function to Run ####
##############################
if __name__ == '__main__':
	# Initialize Node
	rospy.init_node('ros_plane_path_manager')

	# # set rate
	# hz = 10.0
	# rate = rospy.Rate(hz)

	# init path_manager_base object
	manager = path_manager_base()

	rospy.spin()

	# # Loop
	# while not rospy.is_shutdown():
	# 	rate.sleep()