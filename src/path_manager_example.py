#!/usr/bin/env python

import path_manager_base
import numpy as np
import math as *

M_PI_F = 3.14159265358979323846
M_PI_2_F = 1.57079632679489661923

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
			val = fmod(inpt, 2*M_PI_F)
		else:
			n = 0.0
			n = floor(inpt/2/M_PI_F)
			val = inpt - n*2*M_PI_F
		return val

	def dubinsParameters(self, start_node, end_node, R):
		ell = sqrt((start_node.w[0] - end_node.w[0])*(start_node.w[1] - end_node.w[1]))
		if (ell < 2*R):
			pass
		else:
			self._dubinspath.ps[0] = start_node.w[0]
			self._dubinspath.ps[1] = start_node.w[1]
			self._dubinspath.ps[2] = start_node.w[2]
			self._dubinspath.chis = start_node.chi_d
			self._dubinspath.pe[0] = end_node.w[0]
			self._dubinspath.pe[1] = end_node.w[1]
			self._dubinspath.pe[2] = end_node.w[2]
			self._dubinspath.chie = end_node.chi_d

			crs = _dubinspath.ps
			crs[0] += R*(cos(M_PI_2_F)*cos(self._dubinspath.chis) - sin(M_PI_2_F)*sin(self._dubinspath.chis))
			crs[1] += R*(sin(M_PI_2_F)*cos(self._dubinspath.chis) + cos(M_PI_2_F)*sin(self._dubinspath.chis))
			clss = _dubinspath.ps;
			clss[0] += R*(cos(-M_PI_2_F)*cos(self._dubinspath.chis) - sin(-M_PI_2_F)*sin(self._dubinspath.chis))
			clss[1] += R*(sin(-M_PI_2_F)*cos(self._dubinspath.chis) + cos(-M_PI_2_F)*sin(self._dubinspath.chis))
			cre = _dubinspath.pe;
			cre[0] += R*(cos(M_PI_2_F)*cos(self._dubinspath.chie) - sin(M_PI_2_F)*sin(self._dubinspath.chie))
			cre[1] += R*(sin(M_PI_2_F)*cos(self._dubinspath.chie) + cos(M_PI_2_F)*sin(self._dubinspath.chie))
			cle = _dubinspath.pe;
			cle[0] += R*(cos(-M_PI_2_F)*cos(self._dubinspath.chie) - sin(-M_PI_2_F)*sin(self._dubinspath.chie))
			cle[1] += R*(sin(-M_PI_2_F)*cos(self._dubinspath.chie) + cos(-M_PI_2_F)*sin(self._dubinspath.chie))

			# compute L1
			theta = atan2(cre[1] - crs[1], cre[0] - crs[0])
			L1 = (crs - cre).size() + R*self.mo(2*M_PI_F + self.mo(theta - M_PI_2_F) - self.mo(self._dubinspath.chis - M_PI_2_F))
                + R*self.mo(2*M_PI_F + self.mo(self._dubinspath.chie - M_PI_2_F) - self.mo(theta - M_PI_2_F))

            # compute L2
			ell = (cle - crs).size()
			theta = atan2(cle[1] - crs[1], cle[0] - crs[0])
			if(2*R/ell > 1.0 || 2*R/ell < -1.0):
				L2 = 9999.0;
			else:
				theta2 = theta - M_PI_2_F + asin(2*R/ell);
				L2 = sqrt(ell*ell - 4*R*R) + R*self.mo(2*M_PI_F + self.mo(theta2) - self.mo(self._dubinspath.chis - M_PI_2_F))
				    + R*self.mo(2*M_PI_F + self.mo(theta2 + M_PI_F) - self.mo(self._dubinspath.chie + M_PI_2_F));

		    # compute L3
			ell = (cre - clss).size()
			theta = atan2(cre[1] - clss[1], cre[0] - clss[0])
			if (2*R/ell > 1.0 || 2*R/ell < -1.0):
				L3 = 9999.0
			else:
				theta2 = acos(2*R/ell)
				L3 = sqrt(ell*ell - 4*R*R) + R*self.mo(2*M_PI_F + self.mo(self._dubinspath.chis + M_PI_2_F) - self.mo(theta + theta2))
			        + R*self.mo(2*M_PI_F + self.mo(self._dubinspath.chie - M_PI_2_F) - self.mo(theta + theta2 - M_PI_F))

			# compute L4
			theta = atan2(cle[1]-clss[1],cle[0]-clss[0])
			L4 = (clss - cle).size() + R*self.mo(2*M_PI_F + self.mo(self._dubinspath.chis + M_PI_2_F) - mo(theta + M_PI_2_F))
			    + R*self.mo(2*M_PI_F + self.mo(theta + M_PI_2_F) - self.mo(self._dubinspath.chie + M_PI_2_F))

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

			e1 = np.array([0.0, 0.0, 0.0])

			# cpp = switch statement here
			if idx == 1:
				self._dubinspath.cs = crs;
				self._dubinspath.lams = 1;
				self._dubinspath.ce = cre;
				self._dubinspath.lame = 1;
				self._dubinspath.q1 = (cre - crs).normalized();
				self._dubinspath.w1 = self._dubinspath.cs + (self.rotz(-M_PI_2_F)*self._dubinspath.q1)*R;
				self._dubinspath.w2 = self._dubinspath.ce + (self.rotz(-M_PI_2_F)*self._dubinspath.q1)*R;
			elif idx = 2:
				###
			elif idx = 3:
				###
			elif idx = 4:
				###