#include <mavros/mavros_plugin.h>
#include <mavros_msgs/voliro_pid_tuning.h>


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
		mavlink::common::msg:: msg_out;

		msg_out.valid    = msg_in->valid;
	  msg_out.XY_POS_P = msg_in->XY_POS_P;
	  msg_out.XY_POS_I = msg_in->XY_POS_I;
	  msg_out.XY_POS_D = msg_in->XY_POS_D;
	  msg_out.Z_POS_P  = msg_in->Z_POS_P;
	  msg_out.Z_POS_I  = msg_in->Z_POS_I;
	  msg_out.Z_POS_D  = msg_in->Z_POS_D;
	  msg_out.XY_VEL_P = msg_in->XY_VEL_P;
	  msg_out.XY_VEL_I = msg_in->XY_VEL_I;
	  msg_out.XY_VEL_D = msg_in->XY_VEL_D;
	  msg_out.Z_VEL_P  = msg_in->Z_VEL_P;
	  msg_out.Z_VEL_I  = msg_in->Z_VEL_I;
	  msg_out.Z_VEL_D  = msg_in->Z_VEL_D;

	  msg_out.ROLL_P      = msg_in->ROLL_P;
	  msg_out.ROLL_I      = msg_in->ROLL_I;
	  msg_out.ROLL_D      = msg_in->ROLL_D;
	  msg_out.PITCH_P     = msg_in->PITCH_P;
	  msg_out.PITCH_I     = msg_in->PITCH_I;
	  msg_out.PITCH_D     = msg_in->PITCH_D;
	  msg_out.YAW_P       = msg_in->YAW_P;
	  msg_out.YAW_I       = msg_in->YAW_I;
	  msg_out.YAW_D       = msg_in->YAW_D;
	  msg_out.ROLLRATE_P  = msg_in->ROLLRATE_P;
	  msg_out.ROLLRATE_I  = msg_in->ROLLRATE_I;
	  msg_out.ROLLRATE_D  = msg_in->ROLLRATE_D;
	  msg_out.PITCHRATE_P = msg_in->PITCHRATE_P;
	  msg_out.PITCHRATE_I = msg_in->PITCHRATE_I;
	  msg_out.PITCHRATE_D = msg_in->PITCHRATE_D;
	  msg_out.YAWRATE_P   = msg_in->YAWRATE_P;
	  msg_out.YAWRATE_I   = msg_in->YAWRATE_I;
	  msg_out.YAWRATE_D   = msg_in->YAWRATE_D;

		UAS_FCU(m_uas)->send_message_ignore_drop(msg_out);
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLIROtunePIDPlugin, mavros::plugin::PluginBase)
