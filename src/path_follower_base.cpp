#include "path_follower_base.h"
#include "path_follower_example.h"

namespace rosplane {

  path_follower_base::path_follower_base():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
  {
    _vehicle_state_sub = nh_.subscribe<fcu_common::FW_State>("state", 1, &path_follower_base::vehicle_state_callback, this);
    _current_path_sub = nh_.subscribe<fcu_common::FW_Current_Path>("current_path",1, &path_follower_base::current_path_callback, this);


    nh_private_.param<double>("CHI_INFTY", _params.chi_infty, 1.0472);
    nh_private_.param<double>("K_PATH", _params.k_path, 0.025);
    nh_private_.param<double>("K_ORBIT", _params.k_orbit, 8.0);

    _func = boost::bind(&path_follower_base::reconfigure_callback, this, _1, _2);
    _server.setCallback(_func);

    memset(&_vehicle_state, 0, sizeof(_vehicle_state));
    memset(&_current_path, 0, sizeof(_current_path));
    update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &path_follower_base::update, this);
    controller_commands_pub_ = nh_.advertise<fcu_common::FW_Controller_Commands>("controller_commands",1);
  }

void path_follower_base::update(const ros::TimerEvent &)
{

  struct output_s output;

  follow(_params, _input, output);

  fcu_common::FW_Controller_Commands msg;
  msg.chi_c = output.chi_c;
  msg.Va_c = output.Va_c;
  msg.h_c = output.h_c;
  controller_commands_pub_.publish(msg);
}

void path_follower_base::vehicle_state_callback(const fcu_common::FW_StateConstPtr& msg)
{
  _vehicle_state = *msg;
  _input.pn = _vehicle_state.position[0];               /** position north */
  _input.pe = _vehicle_state.position[1];               /** position east */
  _input.h =  -_vehicle_state.position[2];                /** altitude */
  _input.chi = _vehicle_state.chi;

}

void path_follower_base::current_path_callback(const fcu_common::FW_Current_PathConstPtr& msg)
{
  _current_path = *msg;
  _input.flag = _current_path.flag;
  _input.Va_d = _current_path.Va_d;
  for(int i=0;i<3;i++)
  {
    _input.r_path[i] = _current_path.r[i];
    _input.q_path[i] = _current_path.q[i];
    _input.c_orbit[i] = _current_path.c[i];
  }
  _input.rho_orbit = _current_path.rho;
  _input.lam_orbit = _current_path.lambda_;
}

void path_follower_base::reconfigure_callback(ros_plane::FollowerConfig &config, uint32_t level)
{
  _params.chi_infty = config.CHI_INFTY;
  _params.k_path = config.K_PATH;
  _params.k_orbit = config.K_ORBIT;
}
} //end namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_plane_follower");
  rosplane::path_follower_base* path = new rosplane::path_follower();

  ros::spin();

  return 0;
}
