#include <mavros/mavros_plugin.h>
#include <mavros_msgs/voliro_alpha.h>
#include <mavros_msgs/voliro_ao.h>
#include <mavros_msgs/vologging.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief IMU data publication plugin
 */
class VOLOGGINGPlugin : public plugin::PluginBase {
public:

  VOLOGGINGPlugin() : PluginBase(),
                      volo_nh("~voliro"),
                      has_vologging(false)
  {}

  void initialize(UAS& uas_)
  {
    PluginBase::initialize(uas_);

    vologging_pub = volo_nh.advertise<mavros_msgs::vologging>("vologging", 10);
    volo_nh.param<std::string>("frame_id", frame_id, "base_link");

    // reset has_* flags on connection change
    enable_connection_cb();
  }

  Subscriptions get_subscriptions() {
    return {
             make_handler(&VOLOGGINGPlugin::handle_vologging),
    };
  }

private:

  ros::NodeHandle volo_nh;
  ros::Publisher vologging_pub;

  bool has_vologging;
  std::string frame_id;

void handle_vologging(const mavlink::mavlink_message_t   *msg,
                           mavlink::common::msg::VOLOGGING& logging)
  {
    ROS_INFO_COND_NAMED(!has_vologging,
                        "voliro",
                        "VOLIRO: VOLIRO LOGGING detected!");
    has_vologging = true;

    mavros_msgs::vologging vologging;

    // ports for debbuging
    vologging.header      = m_uas->synchronized_header(frame_id,
                                                            logging.time_boot_ms);
		for(int i=0;i<3;++i){
    vologging.port_A_1[i] = logging.port_A_1[i];
    vologging.port_A_2[i] = logging.port_A_2[i];
    vologging.port_A_3[i] = logging.port_A_3[i];
    vologging.port_A_4[i] = logging.port_A_4[i];
    vologging.port_A_5[i] = logging.port_A_5[i];
    vologging.port_A_6[i] = logging.port_A_6[i];
    vologging.port_A_7[i] = logging.port_A_7[i];
    vologging.port_A_8[i] = logging.port_A_8[i];
    vologging.port_A_9[i] = logging.port_A_9[i];
    vologging.port_B_1[i] = logging.port_B_1[i];
    vologging.port_B_2[i] = logging.port_B_2[i];
    vologging.port_B_3[i] = logging.port_B_3[i];
    vologging.port_B_4[i] = logging.port_B_4[i];
    vologging.port_B_5[i] = logging.port_B_5[i];
    vologging.port_B_6[i] = logging.port_B_6[i];
    vologging.port_B_7[i] = logging.port_B_7[i];
    vologging.port_B_8[i] = logging.port_B_8[i];
    vologging.port_B_9[i] = logging.port_B_9[i];
    /*vologging.port_C_1[i] = logging.port_C_1[i];
    vologging.port_C_2[i] = logging.port_C_2[i];
    vologging.port_C_3[i] = logging.port_C_3[i];
    vologging.port_C_4[i] = logging.port_C_4[i];
    vologging.port_C_5[i] = logging.port_C_5[i];
    vologging.port_C_6[i] = logging.port_C_6[i];
    vologging.port_C_7[i] = logging.port_C_7[i];
    vologging.port_C_8[i] = logging.port_C_8[i];
    vologging.port_C_9[i] = logging.port_C_9[i];
    vologging.port_D_1[i] = logging.port_D_1[i];
    vologging.port_D_2[i] = logging.port_D_2[i];
    vologging.port_D_3[i] = logging.port_D_3[i];
    vologging.port_D_4[i] = logging.port_D_4[i];
    vologging.port_D_5[i] = logging.port_D_5[i];
    vologging.port_D_6[i] = logging.port_D_6[i];
    vologging.port_D_7[i] = logging.port_D_7[i];
    vologging.port_D_8[i] = logging.port_D_8[i];
    vologging.port_D_9[i] = logging.port_D_9[i];*/
}
vologging.port_A_1[3] = 0;//Last component doesn'exist in the mavlink message
vologging.port_A_2[3] = 0;
vologging.port_A_3[3] = 0;
vologging.port_A_4[3] = 0;
vologging.port_A_5[3] = 0;
vologging.port_A_6[3] = 0;
vologging.port_A_7[3] = 0;
vologging.port_A_8[3] = 0;
vologging.port_A_9[3] = 0;
vologging.port_B_1[3] = 0;
vologging.port_B_2[3] = 0;
vologging.port_B_3[3] = 0;
vologging.port_B_4[3] = 0;
vologging.port_B_5[3] = 0;
vologging.port_B_6[3] = 0;
vologging.port_B_7[3] = 0;
vologging.port_B_8[3] = 0;
vologging.port_B_9[3] = 0;

    vologging_pub.publish(vologging);
  }

  void connection_cb(bool connected) override
  {
    has_vologging = false;
  }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLOGGINGPlugin,
                       mavros::plugin::PluginBase)
