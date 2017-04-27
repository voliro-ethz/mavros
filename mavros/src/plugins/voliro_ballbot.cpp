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

    ballbot_fullstate_pub = volo_nh.advertise<mavros_msgs::voliro_ballbot>(
      "voliro_ballbot_fullstate",
      10);
    volo_nh.param<std::string>("frame_id", frame_id, "base_link");
    voliro_sub_full = volo_nh.subscribe("voliro_ballbot_desired_fullstate",
                                     1,
                                     &BALLBOTPlugin::voliro_fullstate_cb,
                                     this);
    voliro_sub_short = volo_nh.subscribe("voliro_ballbot_desired_shortstate",
                                     1,
                                     &BALLBOTPlugin::voliro_shortstate_cb,
                                     this);

    _sending_fullstate =false;
    _sending_shortstate=false;
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
  ros::Publisher  ballbot_fullstate_pub;
  ros::Subscriber voliro_sub_full;
  ros::Subscriber voliro_sub_short;


  bool has_voliro_ballbot;
  std::string frame_id;
  bool _sending_fullstate;
  bool _sending_shortstate;

  void handle_ballbot(
    const mavlink::mavlink_message_t *msg,
    mavlink::common::msg::VOLIRO_BALLBOT_FULLSTATE&
    msg_ballbot_fullstate)
  {
    ROS_INFO_COND_NAMED(!has_voliro_ballbot,
                        "voliro",
                        "VOLIRO: VOLIRO BALLBOT detected!");

    has_voliro_ballbot = true;

    mavros_msgs::voliro_ballbot BallbotFullstate;

    // Sending fullstate msg of hexaballbot to ROS
    BallbotFullstate.header = m_uas->synchronized_header(
      frame_id,
      msg_ballbot_fullstate.
      time_boot_ms);

    BallbotFullstate.thetaX    = msg_ballbot_fullstate.thetaX;
    BallbotFullstate.thetaXdot = msg_ballbot_fullstate.thetaXdot;
    BallbotFullstate.thetaY    = msg_ballbot_fullstate.thetaY;
    BallbotFullstate.thetaYdot = msg_ballbot_fullstate.thetaYdot;
    BallbotFullstate.thetaZ    = msg_ballbot_fullstate.thetaZ;
    BallbotFullstate.thetaZdot = msg_ballbot_fullstate.thetaZdot;
    BallbotFullstate.phiX      = msg_ballbot_fullstate.phiX;
    BallbotFullstate.phiXdot   = msg_ballbot_fullstate.phiXdot;
    BallbotFullstate.phiY      = msg_ballbot_fullstate.phiY;
    BallbotFullstate.phiYdot   = msg_ballbot_fullstate.phiYdot;

    BallbotFullstate.start = msg_ballbot_fullstate.start_enabled;
    BallbotFullstate.stop  = msg_ballbot_fullstate.stop_enabled;
    BallbotFullstate.flag1 = msg_ballbot_fullstate.flag1;
    BallbotFullstate.flag2 = msg_ballbot_fullstate.flag2;


    ballbot_fullstate_pub.publish(BallbotFullstate);
  }

  // Send desired state to px4
  // Just send one topic a messsage other ways an error could occur
  void voliro_fullstate_cb(const mavros_msgs::voliro_ballbot fullstate_in) {
    mavlink::common::msg::VOLIRO_BALLBOT_FULLSTATE fullstate_out;
    //Sending also derivatives as desired state, for better tracking behaviour
    fullstate_out.thetaX    = fullstate_in.thetaX;
    fullstate_out.thetaXdot = fullstate_in.thetaXdot;
    fullstate_out.thetaY    = fullstate_in.thetaY;
    fullstate_out.thetaYdot = fullstate_in.thetaYdot;
    fullstate_out.thetaZ    = fullstate_in.thetaZ;
    fullstate_out.thetaZdot = fullstate_in.thetaZdot;
    fullstate_out.phiX      = fullstate_in.phiX;
    fullstate_out.phiXdot   = fullstate_in.phiXdot;
    fullstate_out.phiY      = fullstate_in.phiY;
    fullstate_out.phiYdot   = fullstate_in.phiYdot;

    fullstate_out.start_enabled = fullstate_in.start;
    fullstate_out.stop_enabled  = fullstate_in.stop;
    fullstate_out.flag1         = fullstate_in.flag1;
    fullstate_out.flag2         = fullstate_in.flag2;

    //Check for double Sending
    _sending_fullstate=true;
    if(_sending_fullstate && _sending_shortstate){
      ROS_WARN_STREAM("Both full and short reference is sent to px4. This could lead to unwanted behaviour");
    }
    UAS_FCU(m_uas)->send_message_ignore_drop(fullstate_out);
  }

  void voliro_shortstate_cb(const mavros_msgs::voliro_ballbot fullstate_in) {
    mavlink::common::msg::VOLIRO_BALLBOT_FULLSTATE fullstate_out;
    //Just sending generalized coordinates to the px4, other values are zero
    fullstate_out.thetaX    = fullstate_in.thetaX;
    fullstate_out.thetaXdot = 0.0f;
    fullstate_out.thetaY    = fullstate_in.thetaY;
    fullstate_out.thetaYdot = 0.0f;
    fullstate_out.thetaZ    = fullstate_in.thetaZ;
    fullstate_out.thetaZdot = 0.0f;
    fullstate_out.phiX      = fullstate_in.phiX;
    fullstate_out.phiXdot   = 0.0f;
    fullstate_out.phiY      = fullstate_in.phiY;
    fullstate_out.phiYdot   = 0.0f;

    fullstate_out.start_enabled = fullstate_in.start;
    fullstate_out.stop_enabled  = fullstate_in.stop;
    fullstate_out.flag1         = fullstate_in.flag1;
    fullstate_out.flag2         = fullstate_in.flag2;

    //Check for double Sending
    _sending_shortstate=true;
    if(_sending_fullstate && _sending_shortstate){
      ROS_WARN_STREAM("Both full and short reference is sent to px4. This could lead to unwanted behaviour");
    }
    UAS_FCU(m_uas)->send_message_ignore_drop(fullstate_out);
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
