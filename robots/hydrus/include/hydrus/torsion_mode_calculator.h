#ifndef TORSION_MODE_CALCULATOR
#define TORSION_MODE_CALCULATOR

#include <iostream>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>
#include <hydrus/util/rbdl_util.h>
#include <aerial_robot_control/control/utils/care.h>

#include <Eigen/Dense>
#include <algorithm>
#include <utility>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>

#include <dynamic_reconfigure/server.h>
#include <hydrus/torsion_modeConfig.h>
#define RECONFITURE_TORSION_MODE_FLAG 0
#define TORSION_CONSTANT 1
#define TORSION_Q_MU 2
#define TORSION_Q_MU_D 3

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class TorsionModeCalculator
{
  public:
    TorsionModeCalculator(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~TorsionModeCalculator();

    void calculate();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    // ros subscribers
    ros::Subscriber joint_sub_;
    ros::Subscriber torsion_joint_sub_;
    // ros publishers
    ros::Publisher eigen_pub_;
    ros::Publisher B_mode_pub_;
    ros::Publisher B_pub_;
    ros::Publisher mode_pub_;
    ros::Publisher K_sys_pub_;

    Model* model_;
    std::vector<unsigned int> torsion_dof_update_order_;
    std::vector<unsigned int> joint_dof_update_order_;

    std::string robot_ns_;
    bool is_floating_;
    int rotor_num_;
    int torsion_num_;
    double torsion_constant_;
    int mode_num_;
    double eigen_eps_;
    double m_f_rate_;
    int link1_rotor_direction_;
    bool is_root_on_fc_;
    bool is_use_rbdl_torsion_B_;

    bool is_publish_lqr_gain_;
    double q_mu_;
    double q_mu_d_;
    ros::Publisher K_lqr_mode_pub_;
    void EigenMatrixFloat32MultiArrayPublish(const ros::Publisher& pub, const Eigen::MatrixXd& mat);

    std::vector<double> torsions_;
    std::vector<double> torsions_d_;
    void torsionJointCallback(const sensor_msgs::JointStateConstPtr& msg);

    std::vector<double> joints_;
    std::vector<double> joints_d_;
    void jointsCallback(const sensor_msgs::JointStateConstPtr& msg);

    dynamic_reconfigure::Server<hydrus::torsion_modeConfig>::CallbackType reconf_func_;
    dynamic_reconfigure::Server<hydrus::torsion_modeConfig>* reconf_server_;
    void cfgCallback(hydrus::torsion_modeConfig& config, uint32_t level);
};


#endif /* ifndef TORSION_MODE_CALCULATOR */
