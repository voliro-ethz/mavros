#include <mavros/mavros_plugin.h>
#include <mavros_msgs/voliro_omega.h>
#include <mavros_msgs/voliro_alpha.h>


namespace mavros {
namespace std_plugins{

class VOLIROInversePlugin : public plugin::PluginBase
{
public:
	VOLIROInversePlugin() : PluginBase(),
													voliro_nh("~voliro")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
    voliro_pub = voliro_nh.advertise<mavros_msgs::voliro_omega>("voliro_ao", 10);

		// reset has_* flags on connection change
		enable_connection_cb();
	}

	Subscriptions get_subscriptions()
	{
		return {
      make_handler(&VOLIROInversePlugin::handle_voliro_inverse),
    };
	}

private:
	ros::NodeHandle voliro_nh;
	ros::Publisher voliro_pub;

	void handle_voliro_inverse(const mavlink::mavlink_message_t *msg,
                     mavlink::common::msg::VOLIRO_INVERSE & vol_F_M)
  {


    mavros_msgs::voliro_omega volomsg;
    voliro_pub.publish(volomsg);
  }



};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLIROInversePlugin, mavros::plugin::PluginBase)
