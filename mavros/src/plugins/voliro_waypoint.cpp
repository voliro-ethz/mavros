#include <mavros/mavros_plugin.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <std_msgs/Bool.h>
#include <mavros_msgs/full_voliro.h>

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
		voliro_sub = voliro_nh.subscribe("waypoint", 1, &VOLWaypointPlugin::voliro_ao_cb, this);
		// takeoff_sub = voliro_nh.subscribe("", 1, &VOLWaypointPlugin::voliro_takeoff_cb, this);
		// landing_sub = voliro_nh.subscribe("landing", 1, &VOLWaypointPlugin::voliro_landing_cb, this);
		// velocity_sub = voliro_nh.subscribe("velocity", 1, &VOLWaypointPlugin::voliro_velocity_cb, this);

	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle voliro_nh;
	ros::Subscriber voliro_sub;
	// ros::Subscriber takeoff_sub;
	// ros::Subscriber landing_sub;
	// ros::Subscriber velocity_sub;

	void voliro_ao_cb(const mavros_msgs::full_voliro::ConstPtr &sp)
	{
		mavlink::common::msg::VOLIRO_FULL_SETPOINT v{};

		Eigen::Affine3d tr;
		tf::poseMsgToEigen(sp->posestamped.pose, tr);

		// Transform quaternion and position to NED Coordinate Frame

		auto p = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

		v.time_boot_ms = sp->posestamped.header.stamp.toNSec() / 1000000;
		v.target_system = 1;
		v.target_component = 1;

		v.takeoff_enabled = sp->takeoff;
		v.landing_enabled = sp->landing;
		v.velocity_enabled = sp->velocity;


		v.x = p.x();
		v.y = p.y();
		v.z = p.z();

		// Is this the correct order of quaternions?

		v.q[0] = q.w();
		v.q[1] = q.x();
		v.q[2] = q.y();
		v.q[3] = q.z();


		// Velocities Later!! TODO

		UAS_FCU(m_uas)->send_message_ignore_drop(v);
	}

	// void voliro_takeoff_cb(const std_msgs::Bool::ConstPtr &tk){
	//
	// }
	//
	// void voliro_landing_cb(const std_msgs::Bool::ConstPtr &land){
	//
	// }
	//
	// void voliro_velocity_cb(const std_msgs::Bool::ConstPtr &vel){
	//
	// }

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLWaypointPlugin, mavros::plugin::PluginBase)
