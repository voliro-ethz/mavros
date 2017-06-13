#include <mavros/mavros_plugin.h>
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
		voliro_sub = voliro_nh.subscribe("waypoint", 1, &VOLWaypointPlugin::voliro_full_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle voliro_nh;
	ros::Subscriber voliro_sub;

	void voliro_full_cb(const mavros_msgs::full_voliro::ConstPtr &sp) {

		mavlink::common::msg::VOLIRO_FULL_SETPOINT v{};

		Eigen::Affine3d tr;
		tf::poseMsgToEigen(sp->posestamped.pose, tr);

		// Eigen::Affine3d wallpoint;
		// tf::poseMsgToEigen(sp->wallpoint.pose, wallpoint);
		//
		// Eigen::Affine3d wallvector;
		// tf::poseMsgToEigen(sp->wallvector.pose, wallvector);

		Eigen::Matrix<double,6,1> tw;
		tf::twistMsgToEigen(sp->twiststamped.twist, tw);

Eigen::Vector3d twa(tw(0),tw(1),tw(2));
Eigen::Vector3d twb(tw(3),tw(4),tw(5));

		// Transform quaternion and position to NED Coordinate Frame

		auto p = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

	auto r = ftf::transform_frame_enu_ned(twa);
	auto s = ftf::transform_frame_enu_ned(twb);
	// auto t = ftf::transform_frame_enu_ned(Eigen::Vector3d(wallpoint.translation()));
	// auto u = ftf::transform_frame_enu_ned(Eigen::Vector3d(wallvector.translation()));

		v.time_boot_ms = sp->posestamped.header.stamp.toNSec() / 1000000;
		v.target_system = 1;
		v.target_component = 1;

		v.takeoff_enabled = sp->takeoff;
		v.landing_enabled = sp->landing;
		v.wall_enabled = sp->wall;
		v.filter_sp_enabled = sp->filter_sp;
		v.rotorguards_tola_enabled  = sp->rotorguards_tola;
		v.headless_enabled=sp->headless;
		v.velocity_enabled = sp->velocity;
		v.reset_integrals = sp->reset_integrals;
		v.unwinding = sp->unwinding;
		v.manual_position = sp->manual_position;
		v.fan_enabled = sp->fan_enabled;
		v.winding = sp->winding;
		v.intuitive_control = sp->intuitive_control;
		v.bird = sp->bird;

		v.x = p.x();
		v.y = p.y();
		v.z = p.z();

		v.q[0] = q.w();
		v.q[1] = q.x();
		v.q[2] = q.y();
		v.q[3] = q.z();


		v.vx= r.x();
		v.vy= r.y();
		v.vz= r.z();

		v.roll_rate= s.x();
		v.pitch_rate= s.y();
		v.yaw_rate= s.z();

		v.wallposition=sp->wallposition;
		v.wallpoint[0]=sp->wallpoint[1];
		v.wallpoint[1]=sp->wallpoint[0]; 	//ENU to NED transformation
		v.wallvector[0]=sp->wallvector[1];
		v.wallvector[1]=sp->wallvector[0];

		UAS_FCU(m_uas)->send_message_ignore_drop(v);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLWaypointPlugin, mavros::plugin::PluginBase)
