#!/usr/bin/env python
# Python implementation of "estimator"
# estimator_base.h/cpp & estimator_example.h/cpp

import rospy
from fcu_common.msg import FW_State, GPS
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import Float32
from math import *
import numpy as np

EARTH_RADIUS = 6378145.0

class estimator_base:

	# Init function
	def __init__(self):
		
		# init stuff

		# Get Params
		self.gps_topic_ = rospy.get_param('gps_topic', '/gps/data')
		self.imu_topic_ = rospy.get_param('imu_topic', '/imu/data')
		self.baro_topic_ = rospy.get_param('gps_topic', '/baro/alt')
		self.airspeed_topic_ = rospy.get_param('airspeed_topic_', '/airspeed/data')
		self.update_rate_ = rospy.get_param('update_rate', 100.0)

		self.params_.Ts = 1.0/self.update_rate_
		self.params_.gravity = 9.8

		self.params_.rho = rospy.get_param('rho', 1.225)
		self.params_.sigma_accel = rospy.get_param('sigma_accel', 0.0245)
		self.params_.sigma_n_gps = rospy.get_param('sigma_n_gps', 0.21)
		self.params_.sigma_e_gps = rospy.get_param('sigma_e_gps', 0.21)
		self.params_.sigma_Vg_gps = rospy.get_param('sigma_Vg_gps', 0.0500)
		self.params_.sigma_course_gps = rospy.get_param('sigma_course_gps', 0.0045)

		# Init subscribers
		self.gps_sub_ = rospy.Subscriber(self.gps_topic_, GPS, self.gpsCallback)
		self.imu_sub_ = rospy.Subscriber(self.imu_topic_, Imu, self.imuCallback)
		self.baro_sub_ = rospy.Subscriber(self.baro_topic_, Float32, self.baroAltCallback)
		self.baro_sub_ = rospy.Subscriber(self.airspeed_topic_, FluidPressure, self.airspeedCallback)

		# Init Timer
		self.update_timer_ = rospy.Timer(rospy.Duration(1.0/self.update_rate_), self.update)

		# Init publishers
		self.vehicle_state_pub_ = rospy.Publisher('state', FW_State, queue_size=10)

		# estimator_example.cpp init values
		self.P_a *= (self.radians(5.0))**2

		self.Q_a[0][0] = 0.00000001
		self.Q_a[1][1] = 0.00000001

		self.P_p[0][0] = .03
		self.P_p[1][1] = .03
		self.P_p[2][2] = .01
		self.P_p[3][3] = self.radians(5.0)
		self.P_p[4][4] = .04
		self.P_p[5][5] = .04
		self.P_p[6][6] = self.radians(5.0)

		self.Q_p *= 0.0001
		self.Q_p[3][3] = 0.000001

	# Subclasses
	class input_s:
		gyro_x = 0.0
		gyro_y = 0.0
		gyro_z = 0.0
		accel_x = 0.0
		accel_y = 0.0
		accel_z = 0.0
		baro_alt = 0.0
		diff_pres = 0.0 
		gps_new = 0.0
		gps_n = 0.0
		gps_e = 0.0
		gps_h = 0.0
		gps_Vg = 0.0
		gps_course = 0.0

	class output_s:
		pn = 0.0
		pe = 0.0
		h = 0.0
		Va = 0.0
		alpha = 0.0
		beta = 0.0
		phi = 0.0
		theta = 0.0
		psi = 0.0
		chi = 0.0 
		p = 0.0
		q = 0.0
		r = 0.0
		Vg = 0.0
		wn = 0.0
		we = 0.0

	class params_s:
		gravity = 0.0
		rho = 0.0
		sigma_accel = 0.0
		sigma_n_gps = 0.0	
		sigma_e_gps = 0.0	
		sigma_Vg_gps = 0.0	
		sigma_course_gps = 0.0	
		Ts = 0.0

	# Class Members

	# private from estimator_example.h
	lpf_a_ = 0.0 	# double
	alpha = 0.0		# float
	alpha1 = 0.0	# float
	N_ = 10 			# int

	lpf_gyro_x = 0.0	# float
	lpf_gyro_y = 0.0	# float
	lpf_gyro_z = 0.0 	# float

	lpf_diff = 0.0
	lpf_accel_x = 0.0	# float
	lpf_accel_y = 0.0	# float
	lpf_accel_z = 0.0 	# float

	phat = 0.0		# float
	qhat = 0.0		# float
	rhat = 0.0		# float
	Vwhat = 0.0		# float
	phihat = 0.0	# float
	thetahat = 0.0	# float
	psihat = 0.0	# float


	#float32 = float, float64 = double
	xhat_a = np.zeros((2,), dtype=np.float32) #2 init as zeros
	P_a = np.identity(2, dtype=np.float32) #2x2 init as Identity

	xhat_p = np.zeros((7,), dtype=np.float32) # 7 init as zeros
	P_p = np.identity(7, dtype=np.float32) # 7x7 init as Identity

	Q_a = np.identity(2, dtype=np.float32) #2x2 init as Identity
	R_accel = 0.0
	f_a = np.zeros((2,), dtype=np.float32) #2
	A_a = np.zeros((2,2), dtype=np.float32) #2x2 zeros
	h_a = 0.0
	C_a = np.zeros((2,), dtype=np.float32) #2 zeros
	L_a = np.zeros((2,), dtype=np.float32) #2

	Q_p = np.identity(7, dtype=np.float32) #7x7 init as Identity
	R_p = np.zeros((7,7), dtype=np.float32) #6x6 init as zeros
	f_p = np.zeros((7,), dtype=np.float32) #7
	A_p = np.zeros((7,7), dtype=np.float32) #7x7
	h_p = 0.0
	C_p = np.zeros((7,), dtype=np.float32) #7
	L_p = np.zeros((7,), dtype=np.float32) #7

	# private from estimator_base
	gps_new_ = True 	# bool
	gps_init_ = False 	# bool
	init_lat_ = 0.0 	# double # initial latitude in degrees
	init_lon_ = 0.0		# double # initial longitude in degrees
	init_alt_ = 0.0 	# float # initial altitude in meters above MSL
	# _baro_init = True	# bool 
	# _init_static = 0.0	# float # initial static pressure (mbar)

	params_ = params_s()
	input_ = input_s()


	# Callback Function
	def update(self, event):
		# print 'Estimator.update'
		output = self.output_s()
		output = self.estimate(self.params_, self.input_, output)
		self.input_.gps_new = False

		msg = FW_State()
		msg.position[0] = output.pn
		msg.position[1] = output.pe
		msg.position[2] = -output.h
		msg.Va = output.Va
		msg.alpha = output.alpha
		msg.beta = output.beta
		msg.phi = output.phi
		msg.theta = output.theta
		msg.psi = output.psi
		msg.chi = output.chi
		msg.p = output.p
		msg.q = output.q
		msg.r = output.r
		msg.Vg = output.Vg
		msg.wn = output.wn
		msg.we = output.we
		msg.quat_valid = False

		# rospy.logwarn("PUBLISH")
		self.vehicle_state_pub_.publish(msg)

	def gpsCallback(self, msg):
		# print 'Estimator.gpsCallback'
		if (not self.gps_init_):
			self.gps_init_ = True
			self.init_alt_ = msg.altitude
			self.init_lat_ = msg.latitude
			self.init_lon_ = msg.longitude
			# print 'GPS INIT VALUES'
			# print self.init_alt_ 
			# print self.init_lat_
			# print self.init_lon_
		else:
			self.input_.gps_n = EARTH_RADIUS*(msg.latitude - self.init_lat_)*np.pi/180.0
			self.input_.gps_e = EARTH_RADIUS*cos(self.init_lat_*np.pi/180.0)*(msg.longitude - self.init_lon_)*np.pi/180.0
			self.input_.gps_h = msg.altitude - self.init_alt_
			self.input_.gps_Vg = msg.speed
			# print 'N, E, H, VG from GPS' 
			# print self.input_.gps_n 
			# print self.input_.gps_e 
			# print self.input_.gps_h 
			# print self.input_.gps_Vg
			if(msg.speed > 0.3):
				self.input_.gps_course = msg.ground_course
			if(msg.fix == True and msg.NumSat >= 4):
				self.input_.gps_new = True

	def imuCallback(self, msg):
		# print 'Estimator.imuCallback'
		self.input_.accel_x = msg.linear_acceleration.x
		self.input_.accel_y = msg.linear_acceleration.y
		self.input_.accel_z = msg.linear_acceleration.z

		self.input_.gyro_x = msg.angular_velocity.x
		self.input_.gyro_y = msg.angular_velocity.y
		self.input_.gyro_z = msg.angular_velocity.z

	def baroAltCallback(self, msg):
		# print 'Estimator.baroAltCallback'
		self.input_.baro_alt = msg.data

	def airspeedCallback(self, msg):
		# print 'Estimator.airspeedCallback'
		self.input_.diff_pres = msg.fluid_pressure

	def estimate(self, params, inpt, output):
		# print 'Estimator.estimate'
		if(self.alpha == 0.0): #initailze stuff that comes from params
			self.R_accel = (params.sigma_accel**2)

			self.R_p[0][0] = (params.sigma_n_gps**2)
			self.R_p[1][1] = (params.sigma_e_gps**2)
			self.R_p[2][2] = (params.sigma_Vg_gps**2)
			self.R_p[3][3] = (params.sigma_course_gps**2)
			self.R_p[4][4] = 0.001
			self.R_p[5][5] = 0.001

			lpf_a = 50
			lpf_a1 = 2.0
			self.alpha = exp(-lpf_a*params.Ts)
			self.alpha1 = exp(-lpf_a1*params.Ts)

		# low pass filter gyros to estimate angular rates
		self.lpf_gyro_x = self.alpha*self.lpf_gyro_x + (1-self.alpha)*inpt.gyro_x
		self.lpf_gyro_y = self.alpha*self.lpf_gyro_y + (1-self.alpha)*inpt.gyro_y
		self.lpf_gyro_z = self.alpha*self.lpf_gyro_z + (1-self.alpha)*inpt.gyro_z

		phat = self.lpf_gyro_x
		qhat = self.lpf_gyro_y
		rhat = self.lpf_gyro_z

		# low pass filter static pressure sensor and invert to esimate altitude
		# lpf_static = alpha1*lpf_static + (1-alpha1)*input.static_pres;
		# float hhat = lpf_static/params.rho/params.gravity;
		# !!! normally we would low pass filter the static pressure but the NAZE already does that
		hhat = inpt.baro_alt

		# low pass filter diff pressure sensor and invert to extimate Va
		self.lpf_diff = self.alpha1*self.lpf_diff + (1-self.alpha1)*inpt.diff_pres
		Vahat = sqrt(2.0/params.rho*self.lpf_diff)

		# if(!std::isfinite(hhat) || hhat < -500 || hhat > 500)
		# {
		#     ROS_WARN("problem 20");
		#     hhat = 10;
		# }

		if(not np.isfinite(Vahat) or Vahat <= 0 or Vahat > 50): # CHANGED FROM 25 to 50 b/c flying @ 30
			# ROS_WARN("problem 21");
			Vahat = 30 #9 # Changed to 30

		# low pass filter accelerometers
		self.lpf_accel_x = self.alpha*self.lpf_accel_x + (1-self.alpha)*inpt.accel_x
		self.lpf_accel_y = self.alpha*self.lpf_accel_y + (1-self.alpha)*inpt.accel_y
		self.lpf_accel_z = self.alpha*self.lpf_accel_z + (1-self.alpha)*inpt.accel_z

		# implement continuous-discrete EKF to estimate roll and pitch angles

		# prediction step
		# float cp # cos(phi)
		# float sp # sin(phi)
		# float tt # tan(thata)
		# float ct # cos(thata)
		# float st # sin(theta)

		for i in range(0,self.N_):
			cp = cos(self.xhat_a[0]) # cos(phi)
			sp = sin(self.xhat_a[0]) # sin(phi)
			tt = tan(self.xhat_a[1]) # tan(thata) THETA?
			ct = cos(self.xhat_a[1]) # cos(thata)

			self.f_a[0] = phat + (qhat*sp + rhat*cp)*tt
			self.f_a[1] = qhat*cp - rhat*sp

			self.A_a = np.zeros((2,2), dtype=np.float32) #2x2 zeros REINIT
			self.A_a[0][0] = (qhat*cp - rhat*sp)*tt
			self.A_a[0][1] = (qhat*sp + rhat*cp)/ct/ct
			self.A_a[1][0] = -qhat*sp - rhat*cp

			self.xhat_a += self.f_a *(params.Ts/self.N_)
			self.P_a += (np.dot(self.A_a,self.P_a) + np.dot(self.P_a,self.A_a.transpose()) + self.Q_a)*(params.Ts/self.N_)

		# measurement updates
		cp = cos(self.xhat_a[0])
		sp = sin(self.xhat_a[0])
		ct = cos(self.xhat_a[1])
		st = sin(self.xhat_a[1])

		I = np.identity(2, dtype=np.float32)

		# x-axis accelerometer
		self.h_a = qhat*Vahat*st + params.gravity*st
		self.C_a = np.zeros((2,), dtype=np.float32) #2 zeros REINITIALIZE
		self.C_a[1] = qhat*Vahat*ct + params.gravity*ct
		# self.L_a = (self.P_a*self.C_a) / (self.R_accel + self.C_a.transpose()*self.P_a*self.C_a) # ???????????? L_a is 2x1 vector????????
		self.L_a = (np.dot(self.P_a,self.C_a)) / (self.R_accel + np.dot(np.dot(self.C_a.transpose(),self.P_a),self.C_a)) # ???????????? L_a is 2x1 vector????????
		# self.P_a = (I - self.L_a*self.C_a.transpose())*self.P_a
		self.P_a = np.dot((I - np.dot(self.L_a,self.C_a.transpose())),self.P_a)
		self.xhat_a += self.L_a *(self.lpf_accel_x - self.h_a) # input.accel_x - h_a);

		# y-axis accelerometer
		self.h_a = rhat*Vahat*ct - phat*Vahat*st - params.gravity*ct*sp
		self.C_a = np.zeros((2,), dtype=np.float32) #2 zeros REINITIALIZE
		self.C_a[0] = -params.gravity*cp*ct
		self.C_a[1] = -rhat*Vahat*st - phat*Vahat*ct + params.gravity*st*sp
		# self.L_a = (self.P_a*self.C_a) / (self.R_accel + self.C_a.transpose()*self.P_a*self.C_a) # ???????????? L_a is 2x1 vector????????
		self.L_a = (np.dot(self.P_a,self.C_a)) / (self.R_accel + np.dot(np.dot(self.C_a.transpose(),self.P_a),self.C_a)) # ???????????? L_a is 2x1 vector????????
		# self.P_a = (I - self.L_a*self.C_a.transpose())*self.P_a
		self.P_a = np.dot((I - np.dot(self.L_a,self.C_a.transpose())),self.P_a)
		self.xhat_a += self.L_a *(self.lpf_accel_y - self.h_a) #input.accel_y - h_a);

		# z-axis accelerometer
		self.h_a = -qhat*Vahat*ct - params.gravity*ct*cp
		self.C_a = np.zeros((2,), dtype=np.float32) #2 zeros REINITIALIZE
		self.C_a[0] = params.gravity*sp*ct
		self.C_a[1] = (qhat*Vahat + params.gravity*cp)*st
		# self.L_a = (self.P_a*self.C_a) / (self.R_accel + self.C_a.transpose()*self.P_a*self.C_a) # ???????????? L_a is 2x1 vector????????
		self.L_a = (np.dot(self.P_a,self.C_a)) / (self.R_accel + np.dot(np.dot(self.C_a.transpose(),self.P_a),self.C_a)) # ???????????? L_a is 2x1 vector????????
		# self.P_a = (I - self.L_a*self.C_a.transpose())*self.P_a
		self.P_a = np.dot((I - np.dot(self.L_a,self.C_a.transpose())),self.P_a)
		self.xhat_a += self.L_a *(self.lpf_accel_z - self.h_a) #input.accel_z - h_a);

		self.check_xhat_a();
		phihat = self.xhat_a[0]
		thetahat = self.xhat_a[1]

		# implement continous-discrete EKF to estimate pn, pe, chi, Vg
		# prediction step
		# float psidot, tmp, Vgdot;
		if(fabs(self.xhat_p[2]) < 0.01):
			self.xhat_p[2] = 0.01; # prevent devide by zero

		for i in range(0, self.N_): #(int i=0;i<N_;i++)
			psidot = (qhat*sin(phihat) + rhat*cos(phihat))/cos(thetahat)
			tmp = -psidot*Vahat*(self.xhat_p[4]*cos(self.xhat_p[6]) + self.xhat_p[5]*sin(self.xhat_p[6]))/self.xhat_p[2]
			Vgdot = ((Vahat*cos(self.xhat_p[6]) + self.xhat_p[4])*(-psidot*Vahat*sin(self.xhat_p[6])) + \
				(Vahat*sin(self.xhat_p[6]) + self.xhat_p[5])*(psidot*Vahat*cos(self.xhat_p[6])))/self.xhat_p[2]

			self.f_p = np.zeros((7,), dtype=np.float32) #7 REINIT
			self.f_p[0] = self.xhat_p[2]*cos(self.xhat_p[3])
			self.f_p[1] = self.xhat_p[2]*sin(self.xhat_p[3])
			self.f_p[2] = Vgdot
			self.f_p[3] = params.gravity/self.xhat_p[2]*tan(phihat)*cos(self.xhat_p[3] - self.xhat_p[6])
			self.f_p[6] = psidot

			self.A_p = np.zeros((7,7), dtype=np.float32) #7x7 REINIT
			self.A_p[0][2] = cos(self.xhat_p[3])
			self.A_p[0][3] = -self.xhat_p[2]*sin(self.xhat_p[3])
			self.A_p[1][2] = sin(self.xhat_p[3])
			self.A_p[1][3] = self.xhat_p[2]*cos(self.xhat_p[3])
			self.A_p[2][2] = -Vgdot/self.xhat_p[2]
			self.A_p[2][4] = -psidot*Vahat*sin(self.xhat_p[6])/self.xhat_p[2]
			self.A_p[2][5] = psidot*Vahat*cos(self.xhat_p[6])/self.xhat_p[2]
			self.A_p[2][6] = tmp
			self.A_p[3][2] = -params.gravity/(self.xhat_p[2]**2)*tan(phihat)*cos(self.xhat_p[3] - self.xhat_p[6])
			self.A_p[3][3] = -params.gravity/self.xhat_p[2]*tan(phihat)*sin(self.xhat_p[3]- self.xhat_p[6])
			self.A_p[3][6] = params.gravity/self.xhat_p[2]*tan(phihat)*sin(self.xhat_p[3] - self.xhat_p[6])

			self.xhat_p += self.f_p *(params.Ts/self.N_)
			# self.P_p += (self.A_p*self.P_p + self.P_p*self.A_p.transpose() + self.Q_p)*(params.Ts/self.N_)
			self.P_p += (np.dot(self.A_p,self.P_p) + np.dot(self.P_p,self.A_p.transpose()) + self.Q_p)*(params.Ts/self.N_)
		
		#    while(xhat_p(3) > radians(180.0f)) xhat_p(3) = xhat_p(3) - radians(360.0f);
		#    while(xhat_p(3) < radians(-180.0f)) xhat_p(3) = xhat_p(3) + radians(360.0f);
		#    if(xhat_p(3) > radians(180.0f) || xhat_p(3) < radians(-180.0f))
		#    {
		#        ROS_WARN("problem 17");
		#        xhat_p(3) = 0;
		#    }

    	# measurement updates
		if(inpt.gps_new):
			I_p = np.identity(7, dtype=np.float32) # 7x7 init as Identity

			# gps North position
			self.h_p = self.xhat_p[0]
			self.C_p = np.zeros((7,), dtype=np.float32) #7 REINIT
			self.C_p[0] = 1
			# self.L_p = (self.P_p*self.C_p) / (self.R_p[0][0] + (self.C_p.transpose()*self.P_p*self.C_p))
			self.L_p = (np.dot(self.P_p,self.C_p)) / (self.R_p[0][0] + (np.dot(np.dot(self.C_p.transpose(),self.P_p),self.C_p)))
			# self.P_p = (I_p - self.L_p*self.C_p.transpose())*self.P_p
			self.P_p = np.dot((I_p - np.dot(self.L_p,self.C_p.transpose())),self.P_p)
			self.xhat_p += self.L_p*(inpt.gps_n - self.h_p)

			# gps East position
			self.h_p = self.xhat_p[1]
			self.C_p = np.zeros((7,), dtype=np.float32) #7 REINIT
			self.C_p[1] = 1
			# self.L_p = (self.P_p*self.C_p) / (self.R_p[1][1] + (self.C_p.transpose()*self.P_p*self.C_p))
			self.L_p = (np.dot(self.P_p,self.C_p)) / (self.R_p[1][1] + (np.dot(np.dot(self.C_p.transpose(),self.P_p),self.C_p)))
			# self.P_p = (I_p - self.L_p*self.C_p.transpose())*self.P_p
			self.P_p = np.dot((I_p - np.dot(self.L_p,self.C_p.transpose())),self.P_p)
			self.xhat_p += self.L_p*(inpt.gps_e - self.h_p)

			# gps ground speed
			self.h_p = self.xhat_p[2]
			self.C_p = np.zeros((7,), dtype=np.float32) #7 REINIT
			self.C_p[2] = 1
			# self.L_p = (self.P_p*self.C_p) / (self.R_p[2][2] + (self.C_p.transpose()*self.P_p*self.C_p))
			self.L_p = (np.dot(self.P_p,self.C_p)) / (self.R_p[2][2] + (np.dot(np.dot(self.C_p.transpose(),self.P_p),self.C_p)))
			# self.P_p = (I_p - self.L_p*self.C_p.transpose())*self.P_p
			self.P_p = np.dot((I_p - np.dot(self.L_p,self.C_p.transpose())),self.P_p)
			self.xhat_p += self.L_p*(inpt.gps_Vg - self.h_p)

			# gps course
			if(inpt.gps_Vg > 1):
				#wrap course measurement
				gps_course = fmod(inpt.gps_course, self.radians(360.0))

				while(gps_course - self.xhat_p[3] > self.radians(180.0)):
					gps_course = gps_course - self.radians(360.0)
				while(gps_course - self.xhat_p[3] < self.radians(-180.0)):
					gps_course = gps_course + self.radians(360.0)
				self.h_p = self.xhat_p[3]
				self.C_p = np.zeros((7,), dtype=np.float32) #7 REINIT
				self.C_p[3] = 1
				# self.L_p = (self.P_p*self.C_p) / (self.R_p[3][3] + (self.C_p.transpose()*self.P_p*self.C_p))
				self.L_p = (np.dot(self.P_p,self.C_p)) / (self.R_p[3][3] + np.dot(np.dot(self.C_p.transpose(),self.P_p),self.C_p))
				# self.P_p = (I_p - self.L_p*self.C_p.transpose())*self.P_p
				self.P_p = np.dot((I_p - np.dot(self.L_p,self.C_p.transpose())),self.P_p)
				self.xhat_p += self.L_p*(gps_course - self.h_p)
			
			# // pseudo measurement #1 y_1 = Va*cos(psi)+wn-Vg*cos(chi)
			# h_p = Vahat*cosf(xhat_p(6)) + xhat_p(4) - xhat_p(2)*cosf(xhat_p(3));  // pseudo measurement
			# C_p = Eigen::VectorXf::Zero(7);
			# C_p(2) = -cos(xhat_p(3));
			# C_p(3) = xhat_p(2)*sinf(xhat_p(3));
			# C_p(4) = 1;
			# C_p(6) = -Vahat*sinf(xhat_p(6));
			# L_p = (P_p*C_p) / (R_p(4,4) + (C_p.transpose()*P_p*C_p));
			# P_p = (I_p - L_p*C_p.transpose())*P_p;
			# xhat_p = xhat_p + L_p*(0 - h_p);

			# // pseudo measurement #2 y_2 = Va*sin(psi) + we - Vg*sin(chi)
			# h_p = Vahat*sinf(xhat_p(6))+xhat_p(5)-xhat_p(2)*sinf(xhat_p(3));  // pseudo measurement
			# C_p = Eigen::VectorXf::Zero(7);
			# C_p(2) = -sin(xhat_p(3));
			# C_p(3) = -xhat_p(2)*cosf(xhat_p(3));
			# C_p(5) = 1;
			# C_p(6) = Vahat*cosf(xhat_p(6));
			# L_p = (P_p*C_p) / (R_p(5,5) + (C_p.transpose()*P_p*C_p));
			# P_p = (I_p - L_p*C_p.transpose())*P_p;
			# xhat_p = xhat_p + L_p*(0 - h_p);

			if(self.xhat_p[0] > 10000 or self.xhat_p[0] < -10000):
				rospy.logwarn("gps n problem")
				self.xhat_p[0] = inpt.gps_n
			if(self.xhat_p[1] > 10000 or self.xhat_p[1] < -10000):
				rospy.logwarn("gps e problem")
				self.xhat_p[1] = inpt.gps_e

			# if(xhat_p(2) > 35 || xhat_p(2) < 0)
			# {
			#     ROS_WARN("problem 13");
			#     xhat_p(2) = input.gps_Vg;
			# }
			# if(xhat_p(3) > radians(720.0f) || xhat_p(3) < radians(-720.0f))
			# {
			#     ROS_WARN("problem 14");
			#     xhat_p(3) = input.gps_course;
			# }
			# if(xhat_p(6) > radians(720.0f) || xhat_p(6) < radians(-720.0f))
			# {
			#     ROS_WARN("problem 15");
			#     xhat_p(6) = input.gps_course;
			# }
		problem = False

		print 'xhat_p'
		print self.xhat_p
		# int prob_index
		for i in range(0,7): #(int i=0;i<7;i++)
			if(not np.isfinite(self.xhat_p[i])):
				if(not problem):
					problem = True
					prob_index = i
				# switch(i)
				if (i == 0):
					self.xhat_p[i] = inpt.gps_n
				elif (i == 1):
					self.xhat_p[i] = inpt.gps_e
				elif (i == 2):
					self.xhat_p[i] = inpt.gps_Vg
				elif (i == 3):
					self.xhat_p[i] = inpt.gps_course
				elif (i == 6):
					self.xhat_p[i] = inpt.gps_course
				else:
					self.xhat_p[i] = 0;
				
				self.P_p = np.identity(7, dtype=np.float32) # 7x7 REINIT
				self.P_p[0][0] = .03
				self.P_p[1][1] = .03
				self.P_p[2][2] = .01
				self.P_p[3][3] = self.radians(5.0)
				self.P_p[4][4] = .04
				self.P_p[5][5] = .04
				self.P_p[6][6] = self.radians(5.0)
		if(problem):
			rospy.logwarn("problem 10 %d %d", prob_index, (1 if inpt.gps_new else 0)) #(inpt.gps_new ? 1 : 0))
		if(self.xhat_p[6] - self.xhat_p[3] > self.radians(360.0) or self.xhat_p[6] - self.xhat_p[3] < self.radians(-360.0)):
			# xhat_p(3) = fmodf(xhat_p(3),radians(360.0f));
			self.xhat_p[6] = fmod(self.xhat_p[6],np.pi)

		pnhat = self.xhat_p[0]
		pehat = self.xhat_p[1]
		Vghat = self.xhat_p[2]
		chihat = self.xhat_p[3]
		wnhat = self.xhat_p[4]
		wehat = self.xhat_p[5]
		psihat = self.xhat_p[6]

		output.pn = pnhat
		output.pe = pehat
		output.h = hhat
		output.Va = Vahat
		output.alpha = 0
		output.beta = 0
		output.phi = phihat
		output.theta = thetahat
		output.chi = chihat
		output.p = phat
		output.q = qhat
		output.r = rhat
		output.Vg = Vghat
		output.wn = wnhat
		output.we = wehat
		output.psi = psihat

		return output

	def check_xhat_a(self):
		# print 'Estimator.check_xhat_a'
		if(self.xhat_a[0] > self.radians(85.0) or self.xhat_a[0] < self.radians(-85.0) or not np.isfinite(self.xhat_a[0])):
			if(not np.isfinite(self.xhat_a[0])):
				self.xhat_a[0] = 0
				self.P_a = np.identity(2, dtype=np.float32) #2x2 REINIT
				self.P_a *= (self.radians(20.0)**2)
				rospy.logwarn("problem 00.0")
			elif(self.xhat_a[0] > self.radians(85.0)):
				self.xhat_a[0] = self.radians(82.0)
				rospy.logwarn("problem 00.1")
			elif(self.xhat_a[0] < self.radians(-85.0)):
				self.xhat_a[0] = self.radians(-82.0)
				rospy.logwarn("problem 00.2")
		if(self.xhat_a[1] > self.radians(80.0) or self.xhat_a[1] < self.radians(-80.0) or not np.isfinite(self.xhat_a[1])):
			rospy.logwarn("problem 01")
			if(not np.isfinite(self.xhat_a[1])):
				self.xhat_a[1] = 0
				self.P_a = np.identity(2, dtype=np.float32) #2x2 REINIT
				self.P_a *= (self.radians(20.0)**2)
			elif(self.xhat_a[1] > self.radians(80.0)):
				self.xhat_a[1] = self.radians(77.0)
			elif(self.xhat_a[1] < self.radians(-80.0)):
				self.xhat_a[1] = self.radians(-77.0)

	def radians(self, degrees):
		return np.pi*degrees/180.0


##############################
#### Main Function to Run ####
##############################
if __name__ == '__main__':
	# Initialize Node
	rospy.init_node('ros_plane_estimator')

	# init path_manager_base object
	estimator = estimator_base()

	rospy.spin()

