#include <mavros/mavros_plugin.h>
#include <mavros_msgs/voliro_omega.h>
#include <mavros_msgs/voliro_alpha.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/QR>

# define M_PI       3.14159265358979323846  /* pi */

typedef Eigen::VectorXf Vector;
typedef Eigen::MatrixXf Matrix;


namespace mavros {
namespace std_plugins{

class VOLIROInversePlugin : public plugin::PluginBase
{
public:
	VOLIROInversePlugin() : PluginBase(),
													voliro_nh("~voliro"),
													has_voliro_inverse(false)

	{
		A.resize(6,6);
		A.setZero();

		alpha.resize(6,1);
		F_M_des.resize(6,1);

		omega.resize(6,1);
		omega.setZero();
	}

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
    voliro_pub = voliro_nh.advertise<mavros_msgs::voliro_omega>("voliro_omega", 10);

		// reset has_* flags on connection change
		enable_connection_cb();
	}

	Subscriptions get_subscriptions()
	{
		return {
      make_handler(&VOLIROInversePlugin::handle_voliro_inverse),
    };
	}

private:
	ros::NodeHandle voliro_nh;
	ros::Publisher voliro_pub;
	bool has_voliro_inverse;

	Matrix A;
	Vector alpha;
	Vector F_M_des;
	Vector omega;

	void allocationMatrix(Eigen::MatrixXf& alloc, Vector& alpha){
    alloc.resize(6,6);
    float mu = 1.0f;
    float kappa = 0.016f;
    float l = 0.375f;


    alloc << -mu*sin(alpha(0)),                          mu*sin(alpha(1)),                                          mu/2.0f *sin(alpha(2)),                                                            -mu/2.0f*sin(alpha(3)),                                                          -mu/2.0f*sin(alpha(4)),                                             mu/2.0f*sin(alpha(5)),
             0.0f,                                       0.0f,                                                      sqrt(3.0f)/2.0f*mu*sin(alpha(2)),                                                  -mu*sqrt(3.0f)/2.0f*sin(alpha(3)),                                                sqrt(3.0f)/2.0f*mu*sin(alpha(4)),                                     -sqrt(3.0f)/2.0f*mu*sin(alpha(5)),
             -mu*cos(alpha(0)),                          -mu*cos(alpha(1)),                                        -mu*cos(alpha(2)),                                                                  -mu*cos(alpha(3)),                                                               -mu*cos(alpha(4)),                                                -mu*cos(alpha(5)),
             -mu*l*cos(alpha(0))-kappa*sin(alpha(0)),    mu*l*cos(alpha(1))-kappa*sin(alpha(1)),                    mu*l/2.0f*cos(alpha(2))+kappa/2.0f*sin(alpha(2)),                                  -mu*l/2.0f*cos(alpha(3))+kappa/2.0f*sin(alpha(3)),                               -mu*l/2.0f*cos(alpha(4))+kappa/2.0f*sin(alpha(4)),                     mu*l/2.0f*cos(alpha(5))+kappa/2.0f*sin(alpha(5)),
             0.0f,                                       0.0f,                                                      sqrt(3.0f)/2.0f*mu*l*cos(alpha(2))+sqrt(3.0f)/2.0f*kappa*sin(alpha(2)),            -mu*l*sqrt(3.0f)/2.0f*cos(alpha(3))+kappa*sqrt(3.0f)/2.0f*sin(alpha(3)),          sqrt(3.0f)/2.0f*mu*l*cos(alpha(4))-sqrt(3.0f)/2.0f*kappa*sin(alpha(4)),     -sqrt(3.0f)/2.0f*mu*l*cos(alpha(5))-sqrt(3.0f)/2.0f*kappa*sin(alpha(5)),
             mu*l*sin(alpha(0))-kappa*cos(alpha(0)),     mu*l*sin(alpha(1))+kappa*cos(alpha(1)),                    mu*l*sin(alpha(2))-kappa*cos(alpha(2)),                                             mu*l*sin(alpha(3))+kappa*cos(alpha(3)),                                          mu*l*sin(alpha(4))+kappa*cos(alpha(4)),                         mu*l*sin(alpha(5))-kappa*cos(alpha(5))  ;
}

	void handle_voliro_inverse(const mavlink::mavlink_message_t *msg,
                     mavlink::common::msg::VOLIRO_INVERSE & vol_a_F_M)
  {

		ROS_INFO_COND_NAMED(!has_voliro_inverse,
                        "voliro",
                        "VOLIRO: INVERSE INCOMING!");
    has_voliro_inverse = true;

		for (int i = 0; i < 6; ++i){
			alpha(i) = vol_a_F_M.alpha[i];
			F_M_des(i) = vol_a_F_M.F_M_des[i];
		}

		allocationMatrix(A,alpha);

    omega = A.colPivHouseholderQr().solve(F_M_des);

    mavros_msgs::voliro_omega volomsg;

		for (int i = 0; i < 6; ++i){
			volomsg.omega[i] = omega(i);
		}

    voliro_pub.publish(volomsg);
  }

	void connection_cb(bool connected) override
  {
    has_voliro_inverse = false;
  }

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::VOLIROInversePlugin, mavros::plugin::PluginBase)
