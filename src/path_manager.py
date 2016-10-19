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
 	self.input_s.pn = 0.0 # position North
 	self.input_s.pe = 0.0 # position East
 	self.input_s.h = 0.0 # Altitude
 	self.input_s.chi = 0.0 # course angle

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

  	# void vehicle_state_callback(const fcu_common::FW_StateConstPtr& msg);
   #  void new_waypoint_callback(const fcu_common::FW_Waypoint &msg);
   #  void current_path_publish(struct output_s &output);
