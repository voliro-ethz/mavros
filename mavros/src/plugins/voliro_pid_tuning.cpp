#include <mavros/mavros_plugin.h>
#include <mavros_msgs/voliro_pid_tuning.h>
#include <ros/ros.h>

namespace mavros {
namespace std_plugins{

class VOLIROtunePIDPlugin : public plugin::PluginBase
{
public:
	VOLIROtunePIDPlugin() : PluginBase(),
		voliro_nh("~voliro")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		voliro_sub = voliro_nh.subscribe("voliro_pid_tuning", 1, &VOLIROtunePIDPlugin::voliro_pid_cb, this);

	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle voliro_nh;
	ros::Subscriber voliro_sub;

	void voliro_pid_cb(const mavros_msgs::voliro_pid_tuning::ConstPtr &msg_in)
	{
		mavlink::common::msg::VOLIRO_PID_TUNING msg_out;
		msg_out.valid    = msg_in->valid;
	  msg_out.xy_pos_p = msg_in->XY_POS_P;
	  msg_out.xy_pos_i = msg_in->XY_POS_I;
	  msg_out.xy_pos_d = msg_in->XY_POS_D;
	  msg_out.z_pos_p  = msg_in->Z_POS_P;
	  msg_out.z_pos_i  = msg_in->Z_POS_I;
	  msg_out.z_pos_d  = msg_in->Z_POS_D;
	  msg_out.xy_vel_p = msg_in->XY_VEL_P;
	  msg_out.xy_vel_i = msg_in->XY_VEL_I;
	  msg_out.xy_vel_d = msg_in->XY_VEL_D;
	  msg_out.z_vel_p  = msg_in->Z_VEL_P;
	  msg_out.z_vel_i  = msg_in->Z_VEL_I;
	  msg_out.z_vel_d  = msg_in->Z_VEL_D;

	  msg_out.roll_p      = msg_in->ROLL_P;
	  msg_out.roll_i      = msg_in->ROLL_I;
	  msg_out.roll_d      = msg_in->ROLL_D;
	  msg_out.pitch_p     = msg_in->PITCH_P;
	  msg_out.pitch_i     = msg_in->PITCH_I;
	  msg_out.pitch_d     = msg_in->PITCH_D;
	  msg_out.yaw_p       = msg_in->YAW_P;
	  msg_out.yaw_i       = msg_in->YAW_I;
	  msg_out.yaw_d       = msg_in->YAW_D;
	  msg_out.rollrate_p  = msg_in->ROLLRATE_P;
	  msg_out.rollrate_i  = msg_in->ROLLRATE_I;
	  msg_out.rollrate_d  = msg_in->ROLLRATE_D;
	  msg_out.pitchrate_p = msg_in->PITCHRATE_P;
	  msg_out.pitchrate_i = msg_in->PITCHRATE_I;
	  msg_out.pitchrate_d = msg_in->PITCHRATE_D;
	  msg_out.yawrate_p   = msg_in->YAWRATE_P;
	  msg_out.yawrate_i   = msg_in->YAWRATE_I;
	  msg_out.yawrate_d   = msg_in->YAWRATE_D;

		UAS_FCU(m_uas)->send_message_ignore_drop(msg_out);
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLIROtunePIDPlugin, mavros::plugin::PluginBase)
