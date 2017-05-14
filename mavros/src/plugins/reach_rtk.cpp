#include <mavros/mavros_plugin.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen_conversions/eigen_msg.h>

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
		full_gps_sub = voliro_nh.subscribe("gps_fix", 1, &VOLWaypointPlugin::voliro_full_gps_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle voliro_nh;
	ros::Subscriber full_gps_sub;

	void voliro_full_gps_cb(const mavros_msgs::full_gps::ConstPtr &raw_gps) {

    mavlink::common::msg::GPS_RAW_INT gps;
    gps.fix = ra


		UAS_FCU(m_uas)->send_message_ignore_drop(v);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLWaypointPlugin, mavros::plugin::PluginBase)
