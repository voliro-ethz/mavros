#include <mavros/mavros_plugin.h>
#include <mavros_msgs/voliro_alpha.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief IMU data publication plugin
 */
class VOLIROPubPlugin : public plugin::PluginBase {
public:

  VOLIROPubPlugin() : PluginBase(),
                      volo_nh("~voliro"),
                      has_voliro_alpha(false)
  {}

  void initialize(UAS& uas_)
  {
    PluginBase::initialize(uas_);

    voliro_alpha_pub = volo_nh.advertise<mavros_msgs::voliro_alpha>(
      "voliro_alpha",
      10);
    // volo_nh.param<std::string>("frame_id", frame_id, "base_link");

    // reset has_* flags on connection change
    enable_connection_cb();
  }

  Subscriptions get_subscriptions() {
    return {
             make_handler(&VOLIROPubPlugin::handle_voliro_alpha),
    };
  }

private:

  ros::NodeHandle volo_nh;
  ros::Publisher  voliro_alpha_pub;

  bool has_voliro_alpha;

  void handle_voliro_alpha(const mavlink::mavlink_message_t   *msg,
                           mavlink::common::msg::VOLIRO_ALPHA& vola)
  {
    ROS_INFO_COND_NAMED(!has_voliro_alpha,
                        "voliro",
                        "VOLIRO: VOLIRO ALPHA detected!");
    has_voliro_alpha = true;

    // ROS_INFO("VOLIRO WAS HERE");

    mavros_msgs::voliro_alpha volo_alpha_msg;

    volo_alpha_msg.alpha[0] = vola.alpha[0];
    volo_alpha_msg.alpha[1] = vola.alpha[1];
    volo_alpha_msg.alpha[2] = vola.alpha[2];
    volo_alpha_msg.alpha[3] = vola.alpha[3];
    volo_alpha_msg.alpha[4] = vola.alpha[4];
    volo_alpha_msg.alpha[5] = vola.alpha[5];

    // ports for debbuging
    // volo_alpha_msg.header      = m_uas->synchronized_header(frame_id, vola.time_boot_ms);

    voliro_alpha_pub.publish(volo_alpha_msg);
  }

  void connection_cb(bool connected) override
  {
    has_voliro_alpha = false;
  }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLIROPubPlugin,
                       mavros::plugin::PluginBase)
