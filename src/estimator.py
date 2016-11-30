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
		self.gps_sub_ = rospy.Subscriber(gps_topic_, GPS, self.gpsCallback)
		self.imu_sub_ = rospy.Subscriber(imu_topic_, Imu, self.imuCallback)
		self.baro_sub_ = rospy.Subscriber(baro_topic_, Float32, self.baroAltCallback)
		self.baro_sub_ = rospy.Subscriber(airspeed_topic_, FluidPressure, self.airspeedCallback)

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
	R_p = np.zeros((6,6), dtype=np.float32) #6x6 init as zeros
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


	# Callback Function
	def update(self, event):
		output = output_s()
		output = estimate(self.params_, self.input_, output)
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

		self.vehicle_state_pub_.publish(msg)

	def gpsCallback(self, msg):
		if (!self.gps_init_):
			self.gps_init_ = True
			self.init_alt_ = msg.altitude
			self.init_lat_ = msg.latitude
			self.init_lon_ = msg.longitude
		else:
			self.input_.gps_n = EARTH_RADIUS*(msg.latitude - self.init_lat_)*np.pi/180.0
			self.input_.gps_e = EARTH_RADIUS*cos(self.init_lat_*np.pi/180.0)*(msg.longitude - self.init_lon_)*np.pi/180.0
			self.input_.gps_h = msg.altitude - self.init_alt_
			self.input_.gps_Vg = msg.speed
			if(msg.speed > 0.3):
				self.input_.gps_course = msg.ground_course
			if(msg.fix == True && msg.NumSat >= 4):
				self.input_.gps_new = True

	def imuCallback(self, msg):
		self.input_.accel_x = msg.linear_acceleration.x
		self.input_.accel_y = msg.linear_acceleration.y
		self.input_.accel_z = msg.linear_acceleration.z

		self.input_.gyro_x = msg.angular_velocity.x
		self.input_.gyro_y = msg.angular_velocity.y
		self.input_.gyro_z = msg.angular_velocity.z

	def baroAltCallback(self, msg):
		self.input_.baro_alt = msg.data

	def airspeedCallback(self, msg):
		self.input_.diff_pres = msg.fluid_pressure

	def estimate(self, params, inpt, output):
		if(self.alpha == 0.0): #initailze stuff that comes from params
			self.R_accel = (params.sigma_accel**2)

			self.R_p[0][0] = (self.params.sigma_n_gps**2)
			self.R_p[1][1] = (self.params.sigma_e_gps**2)
			self._p[2][2] = (self.params.sigma_Vg_gps**2)
			self.R_p[3][3] = (self.params.sigma_course_gps**2)
			self.R_p[4][4] = 0.001
			self.R_p[5][5] = 0.001

			lpf_a = 50
			lpf_a1 = 2.0
			self.alpha = exp(-lpf_a*self.params.Ts)
			self.alpha1 = exp(-lpf_a1*self.params.Ts)

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
		Vahat = sqrt(2.0/self.params.rho*self.lpf_diff)

		# if(!std::isfinite(hhat) || hhat < -500 || hhat > 500)
		# {
		#     ROS_WARN("problem 20");
		#     hhat = 10;
		# }

		if(!std::isfinite(Vahat) || Vahat <= 0 || Vahat > 25)
			# ROS_WARN("problem 21");
			Vahat = 9 # WHY IS THIS HERE??????

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

			self.A_a[0][0] = (qhat*cp - rhat*sp)*tt
			self.A_a[0][1] = (qhat*sp + rhat*cp)/ct/ct
			self.A_a[1][0] = -qhat*sp - rhat*cp

			self.xhat_a += self.f_a *(self.params.Ts/self.N_)
			self.P_a += (self.A_a*self.P_a + self.P_a*self.A_a.transpose() + self.Q_a)*(self.params.Ts/self.N_)

		# measurement updates
		cp = cos(self.xhat_a[0])
		sp = sin(self.xhat_a[0])
		ct = cos(self.xhat_a[1])
		st = sin(self.xhat_a[1])

		I = np.identity(2, dtype=np.float32)

		# x-axis accelerometer
		self.h_a = qhat*Vahat*st + self.params.gravity*st
		self.C_a(1) = qhat*Vahat*ct + self.params.gravity*ct
		self.L_a = (self.P_a*self.C_a) / (self.R_accel + self.C_a.transpose()*self.P_a*self.C_a) # ???????????? L_a is 2x1 vector????????
		self.P_a = (I - self.L_a*self.C_a.transpose())*self.P_a
		self.xhat_a += self.L_a *(self.lpf_accel_x - self.h_a); # input.accel_x - h_a);

		# LINE 150
		return output

	def check_xhat_a(self):
		pass

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

