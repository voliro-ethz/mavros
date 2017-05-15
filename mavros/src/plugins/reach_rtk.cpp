#include <mavros/mavros_plugin.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen_conversions/eigen_msg.h>

namespace mavros {
namespace std_plugins{

class VOLReachPlugin : public plugin::PluginBase
{
public:
	VOLReachPlugin() : PluginBase(),
		voliro_nh("~voliro")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		full_gps_sub = voliro_nh.subscribe("full_gps", 1, &VOLReachPlugin::voliro_full_gps_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle voliro_nh;
	ros::Subscriber full_gps_sub;

	void voliro_full_gps_cb(const mavros_msgs::full_gps::ConstPtr &raw_gps) {

		mavlink::common::msg VOLIRO_GPS gps_out;
		gps_out.time_usec = raw_gps->fix.header.stamp.toNSec() / 1000;
		gps_out.fix_type = raw_gps->fix.status.status + 1;
		gps_out.lat = raw_gps->fix.latitude * 10000000; // [degrees * 1E7]
		gps_out.lon = raw_gps->fix.longitude * 10000000; // [degrees * 1E7]
		gps_out.alt = raw_gps->fix.altitude * 1000; // [m * 1000] AMSL

		// ??? TODO
		gps_out.eph = 1.0f;
		gps_out.epv = 1.0f;

		// Velocities ! -> Transform

		//  Verify all the Math !!!! TODO

		// Transform velocities to NED   -> Important to note the velocity part
		auto velocity = ftf::transform_frame_enu_ned(Eigen::Vector3d(
						raw_gps->vel.linear.x,
						raw_gps->vel.linear.y,
						raw_gps->vel.linear.z));

		double vn = velocity.x();
		double ve = velocity.y();
		double vd = velocity.z();

		gps_out.vel = sqrt(vn * vn + ve * ve);  // [m/s] !!
		gps_out.vn = vn; 												// [m/s] !!
		gps_out.ve = ve; 												// [m/s] !!
		gps_out.vd = vd; 												// [m/s] !!

		// Hardcode
		gps_out.satellites_visible = 5;

		UAS_FCU(m_uas)->send_message_ignore_drop(v);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLReachPlugin, mavros::plugin::PluginBase)
