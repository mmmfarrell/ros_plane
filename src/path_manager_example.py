#!/usr/bin/env python

import path_manager_base
import numpy as np
import math as *

class path_manager_example(path_manager_base):

	# Init
	def __init__(self):
		path_manager_base.__init__(self)


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
	dubinspath = dubinspath_s()

	# functions
	def manage(self, params, inpt, output):

		if (self._num_waypoints < 2):
			output.flag = true
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
			output.lambdaa = 0
		else:
			################ FIX THISSS###############
			if (_ptr_a->chi_valid):
				self.manage_dubins(params, inpt, output)
			else:
				# manage_line(params,inpt,output)
				self.manage_fillet(params,inpt,output)

	def manage_line(self, params, inpt, output):
		p = np.array([inpt.pn, inpt.pe, -inpt.h])

	def rotz(self, theta):
		R = np.matrix([cos(theta), -sin(theta), 0.0],
						[sin(theta),cos(theta),0.0],
						[0.0, 0.0, 1.0])
		return R

	def mo(self,inpt):
		val = 0.0
		if (in > 0):
			val = fmod(inpt, 2*np.pi)
		else:
			n = 0.0
			n = floor(inpt/2/np.pi)
			val = inpt - n*2*np.pi
		return val

	def dubinsParameters(self, start_node, end_node, R):
		ell = sqrt((start_node.w[0] - end_node.w[0])*(start_node.w[1] - end_node.w[1]))
		if (ell < 2*R):
			pass
		else:
			self._dubinspath.ps(0) = start_node.w[0]