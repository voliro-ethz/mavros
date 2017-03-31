#include <mavros/mavros_plugin.h>
#include <geometry_msgs/PoseStamped.h>

namespace mavros {
namespace std_plugins{

class VOLWaypointPlugin : public plugin::PluginBase
{
public:
	VOLWaypointPlugin() : PluginBase(),
		voliro_nh("~voliro")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		voliro_sub = voliro_nh.subscribe("waypoint", 1, &VOLWaypointPlugin::voliro_ao_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle voliro_nh;
	ros::Subscriber voliro_sub;

	void voliro_ao_cb(const geometry_msgs::PoseStamped::ConstPtr &sp)
	{
		mavlink::common::msg::VOLIRO_FULL_SETPOINT v{};

    v.time_boot_ms = sp->header.stamp.toNSec()/1000;
    v.target_system = 1;
    v.target_component = 1;

    v.q[0] = sp->pose.orientation.x;
    v.q[1] = sp->pose.orientation.y;
    v.q[2] = sp->pose.orientation.z;
    v.q[3] = sp->pose.orientation.w;

    v.x = sp->pose.position.x;
    v.y = sp->pose.position.y;
    v.z = sp->pose.position.z;

		UAS_FCU(m_uas)->send_message_ignore_drop(v);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLWaypointPlugin, mavros::plugin::PluginBase)
