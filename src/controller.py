#!/usr/bin/env python
# Python implementation of 'ros_plane_controller'
# controller_base.h/cpp & controller_example.h/cpp

import rospy
from fcu_common.msg import FW_State, FW_Controller_Commands, Command, FW_Attitude_Commands
from dynamic_reconfigure.server import Server
from ros_plane.cfg import ControllerConfig
from math import *
import numpy as np


class controller_base:

	# Init function
	def __init__(self):
		
		# init stuff

		# Subscribers
		self._vehicle_state_sub = rospy.Subscriber('state', FW_State, self.vehicle_state_callback)
		self._controller_commands_sub = rospy.Subscriber('controller_commands', FW_Controller_Commands, self.controller_commands_callback)

		# memset(&_vehicle_state, 0, sizeof(_vehicle_state))
		# memset(&_controller_commands, 0, sizeof(_controller_commands))

		# Params
		self._params.trim_e = rospy.get_param('TRIM_E', 0.0)
		self._params.trim_a = rospy.get_param('TRIM_A', 0.0)
		self._params.trim_r = rospy.get_param('TRIM_R', 0.0)
		self._params.trim_t = rospy.get_param('TRIM_T', 0.6)
		self._params.pwm_rad_e = rospy.get_param('PWM_RAD_E', 2.3)
		self._params.pwm_rad_a = rospy.get_param('PWM_RAD_A', -1.6)
		self._params.pwm_rad_r = rospy.get_param('PWM_RAD_R', 1.0)
		self._params.alt_toz = rospy.get_param('ALT_TOZ', 20.0)
		self._params.alt_hz = rospy.get_param('ALT_HZ', 10.0)
		self._params.tau = rospy.get_param('TAU', 5.0)
		self._params.c_kp = rospy.get_param('COURSE_KP', 0.7329)
		self._params.c_kd = rospy.get_param('COURSE_KD', 0.0)
		self._params.c_ki = rospy.get_param('COURSE_KI', 0.07)
		self._params.r_kp = rospy.get_param('ROLL_KP', 1.2855)
		self._params.r_kd = rospy.get_param('ROLL_KD', -0.325)
		self._params.r_ki = rospy.get_param('ROLL_KI', 0.0)#0.10f)
		self._params.p_kp = rospy.get_param('PITCH_KP', 1.0)
		self._params.p_kd = rospy.get_param('PITCH_KD', -0.17)
		self._params.p_ki = rospy.get_param('PITCH_KI', 0.0)
		self._params.p_ff = rospy.get_param('PITCH_FF', 0.0)
		self._params.a_p_kp = rospy.get_param('AS_PITCH_KP', -0.0713)
		self._params.a_p_kd = rospy.get_param('AS_PITCH_KD', -0.0635)
		self._params.a_p_ki = rospy.get_param('AS_PITCH_KI', 0.0)
		self._params.a_t_kp = rospy.get_param('AS_THR_KP', 3.2)
		self._params.a_t_kd = rospy.get_param('AS_THR_KD', 0.0)
		self._params.a_t_ki = rospy.get_param('AS_THR_KI', 0.0)
		self._params.a_kp = rospy.get_param('ALT_KP', 0.045)
		self._params.a_kd = rospy.get_param('ALT_KD', 0.0)
		self._params.a_ki = rospy.get_param('ALT_KI', 0.01)
		self._params.b_kp = rospy.get_param('BETA_KP', -0.1164)
		self._params.b_kd = rospy.get_param('BETA_KD', 0.0)
		self._params.b_ki = rospy.get_param('BETA_KI', -0.0037111)
		self._params.max_e = rospy.get_param('MAX_E', 0.610)
		self._params.max_a = rospy.get_param('MAX_A', 0.523)
		self._params.max_r = rospy.get_param('MAX_R', 0.523)
		self._params.max_t = rospy.get_param('MAX_T', 1.0)

		# Dynamic Reconfigure
		#_func = boost::bind(&controller_base::reconfigure_callback, this, _1, _2)
		# _server.setCallback(_func)
		self._server = Server(ControllerConfig, self.reconfigure_callback)

		# Init Publishers
		self._actuators_pub = rospy.Publisher('command', Command, queue_size=10)
		self._att_cmd_pub = rospy.Publisher('attitude_commands', FW_Attitude_Commands, queue_size=10)
		
		# Init Timer
		self._act_pub_timer = rospy.Timer(rospy.Duration(1.0/100.0), self.actuator_controls_publish)

		self._command_recieved = False

		self.state = self.alt_state.TakeOffZone

		self.c_error = 0
		self.c_integrator = 0
		self.r_error = 0
		self.r_integrator = 0
		self.p_error = 0
		self.p_integrator = 0


	# Subclasses
	class input_s:				# all floats
		Ts = 0.0               # time step */
		h = 0.0                # altitude */
		va = 0.0               # airspeed */
		phi = 0.0              # roll angle */
		theta = 0.0            # pitch angle */
		chi = 0.0              # course angle */
		p = 0.0                # body frame roll rate */
		q = 0.0                # body frame pitch rate */
		r = 0.0                # body frame yaw rate */
		Va_c = 0.0             # commanded airspeed (m/s) */
		h_c = 0.0              # commanded altitude (m) */
		chi_c = 0.0            # commanded course (rad) */

	class output_s:			# all floats
		theta_c = 0.0
		delta_e = 0.0
		phi_c = 0.0
		delta_a = 0.0
		delta_r = 0.0
		delta_t = 0.0

	class params_s:				# all doubles
		alt_hz = 0.0           # altitude hold zone */
		alt_toz = 0.0          # altitude takeoff zone */
		tau = 0.0
		c_kp = 0.0
		c_kd = 0.0
		c_ki = 0.0
		r_kp = 0.0
		r_kd = 0.0
		r_ki = 0.0
		p_kp = 0.0
		p_kd = 0.0
		p_ki = 0.0
		p_ff = 0.0
		a_p_kp = 0.0
		a_p_kd = 0.0
		a_p_ki = 0.0
		a_t_kp = 0.0
		a_t_kd = 0.0
		a_t_ki = 0.0
		a_kp = 0.0
		a_kd = 0.0
		a_ki = 0.0
		b_kp = 0.0
		b_kd = 0.0
		b_ki = 0.0
		trim_e = 0.0
		trim_a = 0.0
		trim_r = 0.0
		trim_t = 0.0
		max_e = 0.0
		max_a = 0.0
		max_r = 0.0
		max_t = 0.0
		pwm_rad_e = 0.0
		pwm_rad_a = 0.0
		pwm_rad_r = 0.0

	class alt_state: # Enum class
		TakeOffZone = 0
		ClimbZone = 1
		DescendZone = 2
		AltitudeHoldZone = 3

	# class members
	_params = params_s()
	_controller_commands = FW_Controller_Commands()
	_vehicle_state = FW_State()

	_command_recieved = False # CHECK THIS

	#course_hold(chi_c, chi, r, const struct params_s &params, Ts) = 0.0
	c_error = 0.0
	c_integrator = 0.0

	#roll_hold(phi_c, phi, p, const struct params_s &params, Ts) = 0.0
	r_error = 0.0
	r_integrator = 0.0

	#pitch_hold(theta_c, theta, q, const struct params_s &params, Ts) = 0.0
	p_error = 0.0
	p_integrator = 0.0

	#airspeed_with_pitch_hold(Va_c, Va, const struct params_s &params, Ts) = 0.0
	ap_error = 0.0
	ap_integrator = 0.0
	ap_differentiator = 0.0

	#airspeed_with_throttle_hold(Va_c, Va, const struct params_s &params, Ts) = 0.0
	at_error = 0.0
	at_integrator = 0.0
	at_differentiator = 0.0

	#altitiude_hold(h_c, h, const struct params_s &params, Ts) = 0.0
	a_error = 0.0
	a_integrator = 0.0
	a_differentiator = 0.0

	# Callbacks
	def vehicle_state_callback(self, msg):
		#print 'Controller - vehicle_state_callback'
		self._vehicle_state = msg

	def controller_commands_callback(self, msg):
		#print 'Controller - controller_commands_callback'
		self._command_recieved = True
		self._controller_commands = msg

	def reconfigure_callback(self, config, level):
		#print 'Controller - reconfigure_callback'
		self._params.trim_e = config.TRIM_E
		self._params.trim_a = config.TRIM_A
		self._params.trim_r = config.TRIM_R
		self._params.trim_t = config.TRIM_T

		self._params.c_kp = config.COURSE_KP
		self._params.c_kd = config.COURSE_KD
		self._params.c_ki = config.COURSE_KI

		self._params.r_kp = config.ROLL_KP
		self._params.r_kd = config.ROLL_KD
		self._params.r_ki = config.ROLL_KI

		self._params.p_kp = config.PITCH_KP
		self._params.p_kd = config.PITCH_KD
		self._params.p_ki = config.PITCH_KI
		self._params.p_ff = config.PITCH_FF

		self._params.a_p_kp = config.AS_PITCH_KP
		self._params.a_p_kd = config.AS_PITCH_KD
		self._params.a_p_ki = config.AS_PITCH_KI

		self._params.a_t_kp = config.AS_THR_KP
		self._params.a_t_kd = config.AS_THR_KD
		self._params.a_t_ki = config.AS_THR_KI

		self._params.a_kp = config.ALT_KP
		self._params.a_kd = config.ALT_KD
		self._params.a_ki = config.ALT_KI

		self._params.b_kp = config.BETA_KP
		self._params.b_kd = config.BETA_KD
		self._params.b_ki = config.BETA_KI
		return config

	def convert_to_pwm(self, output):
		#print 'Controller - convert_to_pwm'
		output.delta_e = output.delta_e*self._params.pwm_rad_e
		output.delta_a = output.delta_a*self._params.pwm_rad_a
		output.delta_r = output.delta_r*self._params.pwm_rad_r
		return output

	def actuator_controls_publish(self, event):
		#print 'Controller - actuator_controls_publish'
		input_ = self.input_s()
		input_.h = -self._vehicle_state.position[2]
		input_.va = self._vehicle_state.Va
		input_.phi = self._vehicle_state.phi
		input_.theta = self._vehicle_state.theta
		input_.chi = self._vehicle_state.chi
		input_.p = self._vehicle_state.p
		input_.q = self._vehicle_state.q
		input_.r = self._vehicle_state.r
		input_.Va_c = self._controller_commands.Va_c
		input_.h_c = self._controller_commands.h_c
		input_.chi_c = self._controller_commands.chi_c
		input_.Ts = 0.01

		output = self.output_s()
		if(self._command_recieved == True):
			output = self.control(self._params, input_, output)

			output = self.convert_to_pwm(output)

			actuators = Command()
			# publish actuator controls #

			actuators.normalized_roll = -output.delta_a#(isfinite(output.delta_a)) ? output.delta_a : 0.0f
			actuators.normalized_pitch = -output.delta_e#(isfinite(output.delta_e)) ? output.delta_e : 0.0f
			actuators.normalized_yaw = output.delta_r#(isfinite(output.delta_r)) ? output.delta_r : 0.0f
			actuators.normalized_throttle = output.delta_t#(isfinite(output.delta_t)) ? output.delta_t : 0.0f

			self._actuators_pub.publish(actuators)

			if(self._att_cmd_pub.get_num_connections() > 0):
				attitudes = FW_Attitude_Commands()
				attitudes.phi_c = output.phi_c
				attitudes.theta_c = output.theta_c
				self._att_cmd_pub.publish(attitudes)

	def control(self, params, input_, output):
		#print 'Controller - control'
		output.delta_r = 0 #cooridinated_turn_hold(input_.beta, params, input_.Ts)
		output.phi_c = self.course_hold(input_.chi_c, input_.chi, input_.r, params, input_.Ts)
		output.delta_a = self.roll_hold(output.phi_c, input_.phi, input_.p, params, input_.Ts)

		#printf("%d %d ", (int)(state), (int)(input_.h))
		print 'self.state'
		print self.state
		if (self.state == self.alt_state.TakeOffZone):
			output.delta_a = self.roll_hold(0.0, input_.phi, input_.p, params, input_.Ts)
			output.delta_t = params.max_t
			output.theta_c = 15*3.14/180
			if(input_.h >= params.alt_toz):
				#            warnx("climb")
				self.state = self.alt_state.ClimbZone
				self.ap_error = 0
				self.ap_integrator = 0
				self.ap_differentiator = 0
	   
		elif (self.state == self.alt_state.ClimbZone):
			output.delta_t = params.max_t
			output.theta_c = self.airspeed_with_pitch_hold(input_.Va_c, input_.va, params, input_.Ts)
			if(input_.h >= input_.h_c - params.alt_hz):
				#            warnx("hold")
				self.state = self.alt_state.AltitudeHoldZone
				self.at_error = 0
				self.at_integrator = 0
				self.at_differentiator = 0
				self.a_error = 0
				self.a_integrator = 0
				self.a_differentiator = 0
			elif(input_.h <= params.alt_toz):
				#            warnx("takeoff")
				self.state = self.alt_state.TakeOffZone
		
		elif (self.state == self.alt_state.DescendZone):
			output.delta_t = 0
			output.theta_c = self.airspeed_with_pitch_hold(input_.Va_c, input_.va, params, input_.Ts)
			if(input_.h <= input_.h_c + params.alt_hz):
				#            warnx("hold")
				self.state = self.alt_state.AltitudeHoldZone
				self.at_error = 0
				self.at_integrator = 0
				self.at_differentiator = 0
				self.a_error = 0
				self.a_integrator = 0
				self.a_differentiator = 0

		elif (self.state == self.alt_state.AltitudeHoldZone):
			output.delta_t = self.airspeed_with_throttle_hold(input_.Va_c, input_.va, params, input_.Ts)
			output.theta_c = self.altitiude_hold(input_.h_c, input_.h, params, input_.Ts)
			if(input_.h >= input_.h_c + params.alt_hz):
				#            warnx("desend")
				self.state = self.alt_state.DescendZone
				self.ap_error = 0
				self.ap_integrator = 0
				self.ap_differentiator = 0
			elif(input_.h <= input_.h_c - params.alt_hz):
				#            warnx("climb")
				self.state = self.alt_state.ClimbZone
				self.ap_error = 0
				self.ap_integrator = 0
				self.ap_differentiator = 0

		output.delta_e = self.pitch_hold(output.theta_c, input_.theta, input_.q, params, input_.Ts)
		#printf("%d\n", (int)(100*output.phi_c))

		return output

# 	int controller_example::getstate()
# {
#     int value = -1
#     switch(state)
#     {
#     case alt_state::TakeOffZone:
#         value = 0
#         break
#     case alt_state::ClimbZone:
#         value = 1
#         break
#     case alt_state::AltitudeHoldZone:
#         value = 2
#         break
#     case alt_state::DescendZone:
#         value = 3
#         break
#     }
#     return value
# }

	def course_hold(self, chi_c, chi, r, params, Ts):
		#print 'Controller - course_hold'
		error = chi_c - chi

		self.c_integrator = self.c_integrator + (Ts/2)*(error + self.c_error)

		up = params.c_kp * error
		ui = params.c_ki * self.c_integrator
		ud = params.c_kd * r

		phi_c = self.sat(up + ui + ud, 40*3.14/180, -40*3.14/180)
		if(abs(params.c_ki) >= 0.00001):
			phi_c_unsat = up + ui + ud
			self.c_integrator = self.c_integrator + (Ts/params.c_ki) * (phi_c - phi_c_unsat)
		
		self.c_error = error
		return phi_c

	def roll_hold(self, phi_c, phi, p, params, Ts):
		#print 'Controller - roll_hold'
		error = phi_c - phi

		self.r_integrator = self.r_integrator + (Ts/2.0) * (error + self.r_error)

		up = params.r_kp * error
		ui = params.r_ki * self.r_integrator
		ud = params.r_kd * p

		delta_a = self.sat(up + ui + ud, params.max_a, -params.max_a)
		if(abs(params.r_ki) >= 0.00001):
			delta_a_unsat = up + ui + ud
			self.r_integrator = self.r_integrator + (Ts/params.r_ki) * (delta_a - delta_a_unsat)
		
		self.r_error = error
		return delta_a

	def pitch_hold(self, theta_c, theta, q, params, Ts):
		#print 'Controller - pitch_hold'
		error = theta_c - theta

		self.p_integrator = self.p_integrator + (Ts/2) * (error + self.p_error)

		up = params.p_kp * error
		ui = params.p_ki * self.p_integrator
		ud = params.p_kd * q

		delta_e = self.sat(params.trim_e/params.pwm_rad_e + up + ui + ud, params.max_e, -params.max_e)
		if(abs(params.p_ki) >= 0.00001):
			delta_e_unsat = params.trim_e/params.pwm_rad_e + up + ui + ud
			self.p_integrator = self.p_integrator + (Ts/params.p_ki) * (delta_e - delta_e_unsat)

		self.p_error = error
		return delta_e

	def airspeed_with_pitch_hold(self, Va_c, Va, params, Ts):
		#print 'Controller - airspeed_with_pitch_hold'
		error = Va_c - Va

		self.ap_integrator = self.ap_integrator + (Ts/2) * (error + self.ap_error)
		self.ap_differentiator = (2*params.tau - Ts)/(2*params.tau + Ts)*self.ap_differentiator + (2/(2*params.tau + Ts))*(error - self.ap_error)

		up = params.a_p_kp * error
		ui = params.a_p_ki * self.ap_integrator
		ud = params.a_p_kd * self.ap_differentiator

		theta_c = self.sat(up + ui + ud, 20*3.14/180, -25*3.14/180)
		if(abs(params.a_p_ki) >= 0.00001):
			theta_c_unsat = up + ui + ud
			self.ap_integrator = self.ap_integrator + (Ts/params.a_p_ki) * (theta_c - theta_c_unsat)
		
		self.ap_error = error
		return theta_c

	def airspeed_with_throttle_hold(self, Va_c, Va, params, Ts):
		#print 'Controller - airspeed_with_throttle_hold'
		error = Va_c - Va

		self.at_integrator = self.at_integrator + (Ts/2) * (error + self.at_error)
		self.at_differentiator = (2*params.tau - Ts)/(2*params.tau + Ts)*self.at_differentiator + (2/(2*params.tau + Ts))*(error - self.at_error)

		up = params.a_t_kp * error
		ui = params.a_t_ki * self.at_integrator
		ud = params.a_t_kd * self.at_differentiator

		delta_t = self.sat(params.trim_t + up + ui + ud, params.max_t, 0)
		if(abs(params.a_t_ki) >= 0.00001):
			delta_t_unsat = params.trim_t + up + ui + ud
			self.at_integrator = self.at_integrator + (Ts/params.a_t_ki) * (delta_t - delta_t_unsat)
		
		self.at_error = error
		return delta_t

	def altitiude_hold(self, h_c, h, params, Ts):
		#print 'Controller - altitude_hold'
		error = h_c - h

		self.a_integrator = self.a_integrator + (Ts/2) * (error + self.a_error)
		self.a_differentiator = (2*params.tau - Ts)/(2*params.tau + Ts)*self.a_differentiator + (2/(2*params.tau + Ts))*(error - self.a_error)

		up = params.a_kp * error
		ui = params.a_ki * self.a_integrator
		ud = params.a_kd * self.a_differentiator

		theta_c = self.sat(up + ui + ud, 35*3.14/180, -35*3.14/180)
		if(abs(params.a_ki) >= 0.00001):
			theta_c_unsat = up + ui + ud
			self.a_integrator = self.a_integrator + (Ts/params.a_ki) * (theta_c - theta_c_unsat)
		
		self.at_error = error #self.a_error ?????????????????/
		return theta_c

#float controller_example::cooridinated_turn_hold(float v, const params_s &params, float Ts)
#{
#    #todo finish this if you want...
#    return 0
#}

	def sat(self, value, up_limit, low_limit):
		#print 'Controller - sat'
		# float rVal
		if(value > up_limit):
			rVal = up_limit
		elif(value < low_limit):
			rVal = low_limit
		else:
			rVal = value

		return rVal

##############################
#### Main Function to Run ####
##############################
if __name__ == '__main__':
	
	# Initialize Node
	rospy.init_node('ros_plane_controller')

	# init path_follower object
	controller = controller_base()

	rospy.spin()