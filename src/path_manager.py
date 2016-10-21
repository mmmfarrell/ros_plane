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

#define SIZE_WAYPOINT_ARRAY 20
class path_manager_base:

###### public
	def self.path_manager_base():

	# void
	def self.waypoint_init():
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

###### protected
	# struct waypoint_s
	self.waypoint_s.w = []
	self.waypoint_s.chi_d = 0.0
	self.waypoint_s.chi_valid = True
	self.waypoint_s.Va_d = 0.0

	# struct waypoint_s _waypoints[SIZE_WAYPOINT_ARRAY];
 #    int _num_waypoints;
 #    struct waypoint_s* _ptr_a;

 	# struct input_s
 	class input_s:
	 	self.pn = 0.0 # position North
	 	self.pe = 0.0 # position East
	 	self.h = 0.0 # Altitude
	 	self.chi = 0.0 # course angle

 	# struct output_s
 	self.output_s.flag = True # Inicates strait line or orbital path (true is line, false is orbit)
 	self.output_s.Va_d = 0.0 # Desired airspeed (m/s)
 	self.output_s.r = [] # Vector to origin of straight line path (m)
 	self.output_s.q = [] # Unit vector, desired direction of travel for line path
 	self.output_s.c = [] # Center of orbital path (m)
 	self.output_s.rho = 0.0 # Radius of orbital path (m)
 	self.output_s.lambda = 1 # Direction of orbital path (cw is 1, ccw is -1)

 	# struct param_s
 	self.param_s.R_min = 0.0

 	#     virtual void manage(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

##### private
	# ros::NodeHandle nh_;
 #    ros::NodeHandle nh_private_;

 	# ros::NodeHandle nh_;
  #   ros::NodeHandle nh_private_;
  #   ros::Subscriber _vehicle_state_sub;     /**< vehicle state subscription */
  #   ros::Subscriber _new_waypoint_sub;      /**< new waypoint subscription */
  #   ros::Publisher  _current_path_pub;      /**< controller commands publication */

  	self._vehicle_state = FW_State()

  	def vehicle_state_callback(msg):
  		_vehicle_state = msg
  		inpt = self.input_s()
  		inpt.pn = _vehicle_state.position[0]
  		inpt.pe = _vehicle_state.position[1]
  		inpt.h = -_vehicle_state.position[2]
  		inpt.chi = _vehicle_state.chi

  		# cpp code
	    # struct output_s outputs;
	    # struct params_s params;
	    # manage(params, input, outputs);
	    # current_path_publish(outputs);

  	def new_waypoint_callback(msg):
  		_waypoints[_num_waypoints].w[0]      = msg.w[0]
	    _waypoints[_num_waypoints].w[1]      = msg.w[1]
	    _waypoints[_num_waypoints].w[2]      = msg.w[2]
	    _waypoints[_num_waypoints].chi_d     = msg.chi_d
	    _waypoints[_num_waypoints].chi_valid = msg.chi_valid
	    _waypoints[_num_waypoints].Va_d      = msg.Va_d
	    _num_waypoints+=1

	def current_path_publsih(output):
		current_path = FW_Current_Path()

		current_path.flag = output.flag
		current_path.Va_d = output.Va_d

		for i in range(0,3):
			current_path.r[i] = output.r[i]
			current_path.q[i] = output.q[i]
			current_path.c[i] = output.c[i]

		current_path.rho = output.rho
		current_path.lambda = output.lambda

		_current_path_pub.publish(current_path)


  	# void vehicle_state_callback(const fcu_common::FW_StateConstPtr& msg);
   #  void new_waypoint_callback(const fcu_common::FW_Waypoint &msg);

   #  void current_path_publish(struct output_s &output);
