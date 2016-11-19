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

		# Init subscribers
		self.gps_sub_ = rospy.Subscriber(gps_topic_, GPS, self.gpsCallback)
		self.imu_sub_ = rospy.Subscriber(imu_topic_, Imu, self.imuCallback)
		self.baro_sub_ = rospy.Subscriber(baro_topic_, Float32, self.baroAltCallback)
		self.baro_sub_ = rospy.Subscriber(airspeed_topic_, FluidPressure, self.airspeedCallback)

		# Init Timer
		self.update_timer_ = rospy.Timer(rospy.Duration(1.0/update_rate_), self.update)

		# Init publishers
		self.vehicle_state_pub_ = rospy.Publisher('state', FW_State, queue_size=10)

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
		fgps_course = 0.0

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
	N_ = 0 			# int

	lpf_gyro_x = 0.0	# float
	lpf_gyro_y = 0.0	# float
	lpf_gyro_z = 0.0 	# float

	phat = 0.0		# float
	qhat = 0.0		# float
	rhat = 0.0		# float
	Vwhat = 0.0		# float
	phihat = 0.0	# float
	thetahat = 0.0	# float
	psihat = 0.0	# float

	#float32 = float, float64 = double
	xhat_a = np.zeros((2,), dtype=np.float32) #2
	P_a = np.zeros((2,2), dtype=np.float32) #2x2

	xhat_p = np.zeros((7,), dtype=np.float32) # 7
	P_p = np.zeros((7,7), dtype=np.float32) # 7x7

	Q_a = np.zeros((2,2), dtype=np.float32) #2x2
	R_accel = 0.0
	f_a = np.zeros((2,), dtype=np.float32) #2
	A_a = np.zeros((2,2), dtype=np.float32) #2x2
	h_a = 0.0
	C_a = np.zeros((2,), dtype=np.float32) #2
	L_a = np.zeros((2,), dtype=np.float32) #2

	Q_p = np.zeros((7,7), dtype=np.float32) #7x7
	R_p = np.zeros((6,6), dtype=np.float32) #6x6
	f_p = np.zeros((7,), dtype=np.float32) #7
	A_p = np.zeros((7,7), dtype=np.float32) #7x7
	h_p = 0.0
	C_p = np.zeros((7,), dtype=np.float32) #7
	L_p = np.zeros((7,), dtype=np.float32) #7

	# private from estimator_base
	gps_new_ = True 	# bool
	gps_init_ = True 	# bool
	init_lat_ = 0.0 	# double # initial latitude in degrees
	init_lon_ = 0.0		# double # initial longitude in degrees
	init_alt_ = 0.0 	# float # initial altitude in meters above MSL
	# _baro_init = True	# bool 
	# _init_static = 0.0	# float # initial static pressure (mbar)

	params_ = params_s()
	input_ = input_s()
