#include <hydrus/hydrus_tilted_fully_actuated_torsion_shift_controller.h>

using namespace aerial_robot_control;

void HydrusTiltedFullyActuatedTorsionShiftController::initialize(ros::NodeHandle nh,
                                           ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  HydrusTiltedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  pid_msg_.x.total.resize(motor_num_);
  pid_msg_.y.total.resize(motor_num_);
  pid_msg_.z.total.resize(motor_num_);

  q_mat_.resize(6, motor_num_);
  q_mat_inv_.resize(motor_num_, 6);

  ros::NodeHandle control_nh(nh_, "controller");
  control_nh.param("gain_shift_matrix_pub_interval", gain_shift_matrix_pub_interval_, 0.1);
  gain_shift_matrix_pub_stamp_ = ros::Time::now().toSec();

  K_gain_for_shift_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("K_gain_for_shift", 1);
  B_eom_kernel_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("B_eom_kernel", 1);

  kernel_mix_ratio_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("kernel_mix_ratio", 1, &HydrusTiltedFullyActuatedTorsionShiftController::kernelMixRatioCallback, this);

  K_gain_for_shift_.resize(motor_num_, 6);
  B_eom_kernel_.resize(motor_num_,motor_num_-4);
}

void HydrusTiltedFullyActuatedTorsionShiftController::kernelMixRatioCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  kernel_mix_ratio_ = msg_utils::Float32MultiArray2EigenMatrix(msg);
}

void HydrusTiltedFullyActuatedTorsionShiftController::controlCore()
{
  PoseLinearController::controlCore();

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
      pid_controllers_.at(Y).result(),
      pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;

  Eigen::VectorXd target_thrust_x_term = q_mat_inv_.col(X) * target_acc_cog.x();
  Eigen::VectorXd target_thrust_y_term = q_mat_inv_.col(Y) * target_acc_cog.y();
  Eigen::VectorXd target_thrust_z_term = q_mat_inv_.col(Z) * target_acc_cog.z();

  // constraint x and y
  int index;
  double max_term = target_thrust_x_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - pid_controllers_.at(X).getLimitSum();
  if(residual > 0)
  {
    ROS_DEBUG("the position x control exceed the limit in rotor %d, %f vs %f ", index, max_term, pid_controllers_.at(X).getLimitSum());
    target_thrust_x_term *= (1 - residual / max_term);
  }

  max_term = target_thrust_y_term.cwiseAbs().maxCoeff(&index);
  residual = max_term - pid_controllers_.at(Y).getLimitSum();
  if(residual > 0)
  {
    ROS_DEBUG("the position y control exceed the limit in rotor %d, %f vs %f ", index, max_term, pid_controllers_.at(Y).getLimitSum());
    target_thrust_y_term *= (1 - residual / max_term);
  }

  // constraint z (also  I term)
  max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
  residual = max_term - pid_controllers_.at(Z).getLimitSum();
  if(residual > 0)
  {
    pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getErrI() - residual / q_mat_inv_(index, Z) / pid_controllers_.at(Z).getIGain());
    target_thrust_z_term *= (1 - residual / max_term);
  }

  for(int i = 0; i < motor_num_; i++)
  {
    target_base_thrust_.at(i) = target_thrust_x_term(i) + target_thrust_y_term(i) + target_thrust_z_term(i);

    pid_msg_.x.total.at(i) =  target_thrust_x_term(i);
    pid_msg_.y.total.at(i) =  target_thrust_y_term(i);
    pid_msg_.z.total.at(i) =  target_thrust_z_term(i);
  }

  allocateYawTerm();
}

bool HydrusTiltedFullyActuatedTorsionShiftController::optimalGain()
{
  //wrench allocation matrix
  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  double mass_inv =  1 / robot_model_->getMass();
  Eigen::MatrixXd q_mat = robot_model_->calcWrenchMatrixOnCoG();
  q_mat_.topRows(3) =  mass_inv * q_mat.topRows(3) ;
  q_mat_.bottomRows(3) =  inertia_inv * q_mat.bottomRows(3);
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);

  // perform null space shift for q_mat_inv_
  for (int i = 0; i < q_mat_inv_.cols(); ++i) {
    K_gain_for_shift_.col(i) = q_mat_inv_.col(i);
  }

  Eigen::FullPivLU<Eigen::MatrixXd> B_eom_lu_decomp(q_mat_);
  B_eom_kernel_ = B_eom_lu_decomp.kernel();

  if (kernel_mix_ratio_.rows() == 6 && kernel_mix_ratio_.cols() == B_eom_kernel_.cols()) {
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < B_eom_kernel_.cols(); ++j) {
        q_mat_inv_.col(i) += B_eom_kernel_.col(j) * kernel_mix_ratio_(i,j);
      }
    }
  }

  // prepare rpy gain
  Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
  for (unsigned int i = 0; i < motor_num_; i++) {
    roll_gains_.at(i) = Eigen::Vector3d( torque_allocation_matrix_inv(i,X) * pid_controllers_.at(ROLL).getPGain(),
                                         torque_allocation_matrix_inv(i,X) * pid_controllers_.at(ROLL).getIGain(),
                                         torque_allocation_matrix_inv(i,X) * pid_controllers_.at(ROLL).getDGain());
    pitch_gains_.at(i) = Eigen::Vector3d( torque_allocation_matrix_inv(i,Y) * pid_controllers_.at(PITCH).getPGain(),
                                          torque_allocation_matrix_inv(i,Y) * pid_controllers_.at(PITCH).getIGain(),
                                          torque_allocation_matrix_inv(i,Y) * pid_controllers_.at(PITCH).getDGain());
    yaw_gains_.at(i) = Eigen::Vector3d( torque_allocation_matrix_inv(i,Z) * pid_controllers_.at(YAW).getPGain(),
                                        torque_allocation_matrix_inv(i,Z) * pid_controllers_.at(YAW).getIGain(),
                                        torque_allocation_matrix_inv(i,Z) * pid_controllers_.at(YAW).getDGain());
  }

  // compensation for gyro moment
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, lqi_mode_));
  return true;
}

void HydrusTiltedFullyActuatedTorsionShiftController::publishGain()
{
  HydrusTiltedLQIController::publishGain();

  if (ros::Time::now().toSec() - gain_shift_matrix_pub_stamp_ > gain_shift_matrix_pub_interval_) {
    gain_shift_matrix_pub_stamp_ = ros::Time::now().toSec();
    K_gain_for_shift_pub_.publish(msg_utils::EigenMatrix2Float32MultiArray(K_gain_for_shift_));
    B_eom_kernel_pub_.publish(msg_utils::EigenMatrix2Float32MultiArray(B_eom_kernel_));
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedFullyActuatedTorsionShiftController, aerial_robot_control::ControlBase);
