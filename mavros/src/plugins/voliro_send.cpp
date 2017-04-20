#include <mavros/mavros_plugin.h>
#include <mavros_msgs/voliro_ao.h>
#include <mavros_msgs/voliro_alpha.h>


namespace mavros {
namespace std_plugins{

class VOLIROSendPlugin : public plugin::PluginBase
{
public:
	VOLIROSendPlugin() : PluginBase(),
		voliro_nh("~voliro")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		voliro_sub = voliro_nh.subscribe("voliro_ao", 1, &VOLIROSendPlugin::voliro_ao_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle voliro_nh;
	ros::Subscriber voliro_sub;

	void voliro_ao_cb(const mavros_msgs::voliro_ao::ConstPtr &volo)
	{
		mavlink::common::msg::VOLIRO_AO v;
		mavlink::common::msg::VOLIRO_ALPHA va;

		//ROS_INFO("VOLIRO WAS HERE");

		for (int i = 0; i < 6; ++i){
				v.alpha[i] = volo->alpha[i];
				v.omega[i] = volo->omega[i];

				va.alpha[i] = volo->alpha[i];
		}


		UAS_FCU(m_uas)->send_message_ignore_drop(v);
		UAS_FCU(m_uas)->send_message_ignore_drop(va);
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLIROSendPlugin, mavros::plugin::PluginBase)
