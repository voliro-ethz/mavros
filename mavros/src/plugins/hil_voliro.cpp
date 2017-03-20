#include <mavros/mavros_plugin.h>
// #include <mavros_msgs/voliro_ao.h>
#include <mavros_msgs/hil_gps_msg.h>
#include <mavros_msgs/sensor_msg.h>
#include <mavros_msgs/hil_state_quat.h>

//Conversion of ROS messages to mavlink messages

namespace mavros {
namespace std_plugins{

class HILVoliroPlugin : public plugin::PluginBase
{
public:
	HILVoliroPlugin() : PluginBase(),
		voliro_nh("~voliro")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		//voliro_sub = voliro_nh.subscribe("voliro_ao", 1, &HILVoliroPlugin::voliro_ao_cb, this);
		voliro_sub_hil_gps = voliro_nh.subscribe("hil_gps_msg", 1, &HILVoliroPlugin::hil_gps_msg_cb, this);
		voliro_sub_sensor = voliro_nh.subscribe("sensor_msg", 1, &HILVoliroPlugin::sensor_msg_cb, this);
		voliro_sub_hil_state = voliro_nh.subscribe("hil_state_quat", 1, &HILVoliroPlugin::hil_state_quat_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle voliro_nh;
	ros::Subscriber voliro_sub_hil_gps;
	ros::Subscriber voliro_sub_sensor;
	ros::Subscriber voliro_sub_hil_state;

	// Conversion of voliro_ao (Alphas and Omegas)
	//void voliro_ao_cb(const mavros_msgs::voliro_ao::ConstPtr &volo)
	//{
	//	mavlink::common::msg::VOLIRO_AO v;

	//	for (int i = 0; i < 6; ++i){
	//			v.alpha[i] = volo->alpha[i];
	//			v.omega[i] = volo->omega[i];
	//	}
	//	UAS_FCU(m_uas)->send_message_ignore_drop(v);
	//}

	// Conversion of hil_gps_msg
	void hil_gps_msg_cb(const mavros_msgs::hil_gps_msg::ConstPtr &hil_gps_msg_ros)
	{
		mavlink::common::msg::HIL_GPS hil_gps_msg_mavlink;

		hil_gps_msg_mavlink.time_usec = hil_gps_msg_ros->header.stamp.toNSec()/1000;
		hil_gps_msg_mavlink.fix_type = hil_gps_msg_ros->fix_type;
		hil_gps_msg_mavlink.lat = hil_gps_msg_ros->lat;
		hil_gps_msg_mavlink.lon = hil_gps_msg_ros->lon;
		hil_gps_msg_mavlink.alt = hil_gps_msg_ros->alt;
		hil_gps_msg_mavlink.eph = hil_gps_msg_ros->eph;
		hil_gps_msg_mavlink.epv = hil_gps_msg_ros->epv;
		hil_gps_msg_mavlink.vel = hil_gps_msg_ros->vel;
		hil_gps_msg_mavlink.vn = hil_gps_msg_ros->vn;
		hil_gps_msg_mavlink.ve = hil_gps_msg_ros->ve;
		hil_gps_msg_mavlink.vd = hil_gps_msg_ros->vd;
		hil_gps_msg_mavlink.cog = hil_gps_msg_ros->cog;
		hil_gps_msg_mavlink.satellites_visible = hil_gps_msg_ros->satellites_visible;

		UAS_FCU(m_uas)->send_message_ignore_drop(hil_gps_msg_mavlink);
	}

	// Conversion of sensor_msg
	void sensor_msg_cb(const mavros_msgs::sensor_msg::ConstPtr &sensor_msg_ros)
	{
		mavlink::common::msg::HIL_SENSOR sensor_msg_mavlink;

		sensor_msg_mavlink.time_usec = sensor_msg_ros->header.stamp.toNSec()/1000;
		sensor_msg_mavlink.xacc = sensor_msg_ros->xacc;
		sensor_msg_mavlink.yacc = sensor_msg_ros->yacc;
		sensor_msg_mavlink.zacc = sensor_msg_ros->zacc;
		sensor_msg_mavlink.xgyro = sensor_msg_ros->xgyro;
		sensor_msg_mavlink.ygyro = sensor_msg_ros->ygyro;
		sensor_msg_mavlink.zgyro = sensor_msg_ros->zgyro;
		sensor_msg_mavlink.xmag = sensor_msg_ros->xmag;
		sensor_msg_mavlink.ymag = sensor_msg_ros->ymag;
		sensor_msg_mavlink.zmag = sensor_msg_ros->zmag;
		sensor_msg_mavlink.abs_pressure = sensor_msg_ros->abs_pressure;
		sensor_msg_mavlink.diff_pressure = sensor_msg_ros->diff_pressure;
		sensor_msg_mavlink.pressure_alt = sensor_msg_ros->pressure_alt;
		sensor_msg_mavlink.temperature = sensor_msg_ros->temperature;
		sensor_msg_mavlink.fields_updated = sensor_msg_ros->fields_updated;

		UAS_FCU(m_uas)->send_message_ignore_drop(sensor_msg_mavlink);
	}

	// Conversion of hil_state_quat
	void hil_state_quat_cb(const mavros_msgs::hil_state_quat::ConstPtr &hil_state_quat_ros)
	{
		mavlink::common::msg::HIL_STATE_QUATERNION hil_state_quat_mavlink;

		hil_state_quat_mavlink.time_usec = hil_state_quat_ros->header.stamp.toNSec()/1000;
		for (int i = 0; i<4; i++ ){
			hil_state_quat_mavlink.attitude_quaternion[i] = hil_state_quat_ros->attitude_quaternion[i];
		}
		hil_state_quat_mavlink.rollspeed = hil_state_quat_ros->rollspeed;
		hil_state_quat_mavlink.pitchspeed = hil_state_quat_ros->pitchspeed;
		hil_state_quat_mavlink.yawspeed = hil_state_quat_ros->yawspeed;
		hil_state_quat_mavlink.lat = hil_state_quat_ros->lat;
		hil_state_quat_mavlink.lon = hil_state_quat_ros->lon;
		hil_state_quat_mavlink.alt = hil_state_quat_ros->alt;
		hil_state_quat_mavlink.vx = hil_state_quat_ros->vx;
		hil_state_quat_mavlink.vy = hil_state_quat_ros->vy;
		hil_state_quat_mavlink.vz = hil_state_quat_ros->vz;
		hil_state_quat_mavlink.ind_airspeed = hil_state_quat_ros->ind_airspeed;
		hil_state_quat_mavlink.true_airspeed = hil_state_quat_ros->true_airspeed;
		hil_state_quat_mavlink.xacc = hil_state_quat_ros->xacc;
		hil_state_quat_mavlink.yacc = hil_state_quat_ros->yacc;
		hil_state_quat_mavlink.zacc = hil_state_quat_ros->zacc;

		UAS_FCU(m_uas)->send_message_ignore_drop(hil_state_quat_mavlink);
	}



};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HILVoliroPlugin, mavros::plugin::PluginBase)
