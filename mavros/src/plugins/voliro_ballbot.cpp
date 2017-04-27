#include <mavros/mavros_plugin.h>
// #include <mavros_msgs/voliro_alpha.h>
// #include <mavros_msgs/voliro_ao.h>
#include <mavros_msgs/voliro_ballbot.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief IMU data publication plugin
 */
class BALLBOTPlugin : public plugin::PluginBase {
public:

  BALLBOTPlugin() : PluginBase(),
                      volo_nh("~voliro"),
                      has_voliro_ballbot(false)
  {}

  void initialize(UAS& uas_)
  {
    PluginBase::initialize(uas_);

    ballbot_fullstate_pub = volo_nh.advertise<mavros_msgs::voliro_ballbot>("voliro_ballbot_fullstate", 10);
    volo_nh.param<std::string>("frame_id", frame_id, "base_link");

    // reset has_* flags on connection change
    enable_connection_cb();
  }

  Subscriptions get_subscriptions() {
    return {
             make_handler(&BALLBOTPlugin::handle_ballbot),
    };
  }

private:

  ros::NodeHandle volo_nh;
  ros::Publisher ballbot_fullstate_pub;

  bool has_voliro_ballbot;
  std::string frame_id;

void handle_ballbot(const mavlink::mavlink_message_t   *msg,
                           mavlink::common::msg::VOLIRO_BALLBOT_FULLSTATE& msg_ballbot_fullstate)
  {
    ROS_INFO_COND_NAMED(!has_voliro_ballbot,
                        "voliro",
                        "VOLIRO: VOLIRO BALLBOT detected!");
    has_voliro_ballbot = true;

    mavros_msgs::voliro_ballbot BallbotFullstate;

    // Sending fullstate msg of hexaballbot to ROS
    BallbotFullstate.header = m_uas->synchronized_header(frame_id,  msg_ballbot_fullstate.time_boot_ms);

    BallbotFullstate.thetaX=msg_ballbot_fullstate.thetaX;
    BallbotFullstate.thetaXdot=msg_ballbot_fullstate.thetaXdot;
    BallbotFullstate.thetaY=msg_ballbot_fullstate.thetaY;
    BallbotFullstate.thetaYdot=msg_ballbot_fullstate.thetaYdot;
    BallbotFullstate.thetaZ=msg_ballbot_fullstate.thetaZ;
    BallbotFullstate.thetaZdot=msg_ballbot_fullstate.thetaZdot;
    BallbotFullstate.phiX=msg_ballbot_fullstate.phiX;
    BallbotFullstate.phiXdot=msg_ballbot_fullstate.phiXdot;
    BallbotFullstate.phiY=msg_ballbot_fullstate.phiY;
    BallbotFullstate.phiYdot=msg_ballbot_fullstate.phiYdot;

    BallbotFullstate.start=msg_ballbot_fullstate.start_enabled;
    BallbotFullstate.stop=msg_ballbot_fullstate.stop_enabled;
    BallbotFullstate.flag1=msg_ballbot_fullstate.flag1;
    BallbotFullstate.flag2=msg_ballbot_fullstate.flag2;


    ballbot_fullstate_pub.publish(BallbotFullstate);
  }

  void connection_cb(bool connected) override
  {
    has_voliro_ballbot = false;
  }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::BALLBOTPlugin,
                       mavros::plugin::PluginBase)
