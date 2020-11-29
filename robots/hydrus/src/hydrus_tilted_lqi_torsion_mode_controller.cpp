#include <hydrus/hydrus_tilted_lqi_torsion_mode_controller.h>

using namespace aerial_robot_control;

void HydrusTiltedLQITorsionModeController::initialize(ros::NodeHandle nh,
                                           ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  HydrusLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  desired_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);

  nlopt_throttle_count_=0;
  if (is_debug_) {
    debug_pub_throttle_count_ = 0;
    K_gain_pub_ = nhp_.advertise<std_msgs::Float32MultiArray>("K_gain", 1);
    K_gain_shifted_pub_ = nhp_.advertise<std_msgs::Float32MultiArray>("K_gain_shifted", 1);
    modes_pub_ = nhp_.advertise<std_msgs::Float32MultiArray>("modes", 1);
    modes_d_pub_ = nhp_.advertise<std_msgs::Float32MultiArray>("modes_d", 1);
    B_kernel_pub_ = nhp_.advertise<std_msgs::Float32MultiArray>("B_kernel", 1);
    if (!is_use_rbdl_torsion_B_) {
      torsion_B_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("torsion_B_matrix", 1);
      torsion_B_mode_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("torsion_B_mode_matrix", 1);
    }
  }
  if (is_use_torsion_null_space_shift_) {
    K_gain_shift_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("K_gain_for_shift", 1);
    K_mode_shift_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("K_mode", 1);
    B_eom_kernel_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("B_eom_kernel", 1);
    nlopt_alpha_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("nlopt_alpha", 1);
    kernel_mix_ratio_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("kernel_mix_ratio", 1, &HydrusTiltedLQITorsionModeController::kernelMixRatioCallback, this);
  }

  pid_msg_.z.p_term.resize(1);
  pid_msg_.z.i_term.resize(1);
  pid_msg_.z.d_term.resize(1);

  //rosParamInit();
  q_mu_p_gains_.resize(mode_num_);
  q_mu_d_gains_.resize(mode_num_);
  torsion_num_ = motor_num_-1;
  torsions_.resize(torsion_num_);
  torsions_d_.resize(torsion_num_);
  torsion_eigens_.resize(mode_num_);
  torsion_mode_matrix_.resize(mode_num_, torsion_num_);
  torsion_B_matrix_.resize(mode_num_, motor_num_);
  torsion_B_rot_matrix_.resize(3, motor_num_);
  kernel_mix_ratio_.resize(9);
  for (int i = 0; i < kernel_mix_ratio_.size(); ++i) {
    kernel_mix_ratio_[i].resize(motor_num_, 0.0);
  }

  link_torsion_sub_ = nh_.subscribe<sensor_msgs::JointState>("link_torsion/joint_states", 10, &HydrusTiltedLQITorsionModeController::linkTorsionCallback, this);
  eigen_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("torsion_eigens", 10, &HydrusTiltedLQITorsionModeController::torsionEigensCallback, this);
  mode_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("torsion_mode_matrix", 10, &HydrusTiltedLQITorsionModeController::torsionModeCallback, this);
  if (is_use_rbdl_torsion_B_) {
    B_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("torsion_B_mode_matrix", 10, &HydrusTiltedLQITorsionModeController::torsionBCallback, this);
  }
  if (use_rbdl_b_rot_) {
    B_rot_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("torsion_B_rot_matrix", 10, &HydrusTiltedLQITorsionModeController::torsionBRotCallback, this);
  }

  //dynamic reconfigure server
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle cfgTorsion_nh(control_nh, "lqi_torsion");
  lqi_torsion_server_ = new dynamic_reconfigure::Server<hydrus::LQI_torsionConfig>(cfgTorsion_nh);
  dynamic_reconf_func_lqi_torsion_ = boost::bind(&HydrusTiltedLQITorsionModeController::cfgLQITorsionCallback, this, _1, _2);
  lqi_torsion_server_->setCallback(dynamic_reconf_func_lqi_torsion_);

  // service server
  q_mu_srv_ = nhp_.advertiseService("set_q_mu", &HydrusTiltedLQITorsionModeController::setQMuCallback, this);
  q_mu_d_srv_ = nhp_.advertiseService("set_q_mu_d", &HydrusTiltedLQITorsionModeController::setQMuDCallback, this);

  ros::Duration(init_wait_time_).sleep(); // sleep for a second
}

HydrusTiltedLQITorsionModeController::~HydrusTiltedLQITorsionModeController()
{
  if (lqi_torsion_server_ != nullptr) delete lqi_torsion_server_;
}

void HydrusTiltedLQITorsionModeController::controlCore()
{
  PoseLinearController::controlCore();

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());

  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
  target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));

  Eigen::VectorXd f = robot_model_->getStaticThrust();
  Eigen::VectorXd allocate_scales = f / f.sum() * robot_model_->getMass();
  Eigen::VectorXd target_thrust_z_term = allocate_scales * target_acc_w.length();

  // constraint z (also  I term)
  int index;
  double max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - pid_controllers_.at(Z).getLimitSum();
  if(residual > 0)
    {
      pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getErrI() - residual / allocate_scales(index) / pid_controllers_.at(Z).getIGain());
      target_thrust_z_term *= (1 - residual / max_term);
    }

  for(int i = 0; i < motor_num_; i++)
    {
      target_base_thrust_.at(i) = target_thrust_z_term(i);
      pid_msg_.z.total.at(i) =  target_thrust_z_term(i);
    }

  allocateYawTerm();

  /* torsion suppression control */
  Eigen::VectorXd modes = torsion_mode_matrix_ * Eigen::Map<Eigen::VectorXd>(&torsions_[0], torsions_.size());
  Eigen::VectorXd modes_d = torsion_mode_matrix_ * Eigen::Map<Eigen::VectorXd>(&torsions_d_[0], torsions_d_.size());
  if (is_debug_) {
    std_msgs::Float32MultiArray modes_msg;
    modes_msg.data.clear();
    for (int i = 0; i < modes.size(); ++i) {
        modes_msg.data.push_back(modes(i));
    }
    modes_pub_.publish(modes_msg);

    std_msgs::Float32MultiArray modes_d_msg;
    modes_d_msg.data.clear();
    for (int i = 0; i < modes_d.size(); ++i) {
        modes_d_msg.data.push_back(modes_d(i));
    }
    modes_d_pub_.publish(modes_d_msg);
  }

  for (int i = 0; i < motor_num_; ++i) {
    for (int j = 0; j < mode_num_; ++j) {
      target_base_thrust_[i] += q_mu_p_gains_[j][i] * modes(j) + q_mu_d_gains_[j][i] * modes_d(j);
    }
  }
}

bool HydrusTiltedLQITorsionModeController::optimalGain()
{
  /* use z in calculation */
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(lqi_mode_, motor_num_);
  Eigen::MatrixXd inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  P_dash.row(0) = P.row(2) / robot_model_->getMass(); // z
  P_dash.bottomRows(lqi_mode_ - 1) = (inertia.inverse() * P.bottomRows(3)).topRows(lqi_mode_ - 1); // roll, pitch, yaw

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(lqi_mode_ * 3 +mode_num_*2, lqi_mode_ * 3 +mode_num_*2);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(lqi_mode_ * 3 +mode_num_*2, motor_num_);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(lqi_mode_ +mode_num_, lqi_mode_ * 3 +mode_num_*2);

  Eigen::MatrixXd A_eom = Eigen::MatrixXd::Zero(lqi_mode_ * 3, lqi_mode_ * 3);
  Eigen::MatrixXd B_eom = Eigen::MatrixXd::Zero(lqi_mode_ * 3, motor_num_);
  Eigen::MatrixXd C_eom = Eigen::MatrixXd::Zero(lqi_mode_, lqi_mode_ * 3);
  for(int i = 0; i < lqi_mode_; i++)
    {
      A_eom(2 * i, 2 * i + 1) = 1;
      if (i == 0) {
        B_eom.row(2 * i + 1) = P_dash.row(i);
      } else {
        if (use_rbdl_b_rot_) {
          B_eom.row(2 * i + 1) = torsion_B_rot_matrix_.row(i-1);
        } else {
          B_eom.row(2 * i + 1) = P_dash.row(i);
        }
      }
      C_eom(i, 2 * i) = 1;
    }
  A_eom.block(lqi_mode_ * 2, 0, lqi_mode_, lqi_mode_ * 3) = -C_eom;

  Eigen::FullPivLU<Eigen::MatrixXd> B_eom_lu_decomp(B_eom);
  B_eom_kernel_ = B_eom_lu_decomp.kernel();
  ROS_DEBUG_STREAM("B_eom rows: " << B_eom.rows() );
  ROS_DEBUG_STREAM("B_eom rank: " << B_eom_lu_decomp.rank() );
  ROS_DEBUG_STREAM("B_eom kernel:\n" << B_eom_kernel_ );
  if (is_debug_ && debug_pub_throttle_count_%DEBUG_PUB_THROTTLE_FACTOR) {
    std_msgs::Float32MultiArray B_kernel_msg;
    B_kernel_msg.data.clear();
    for (int i = 0; i < B_eom_kernel_.rows(); ++i) {
      for (int j = 0; j < B_eom_kernel_.cols(); ++j) {
        B_kernel_msg.data.push_back(B_eom_kernel_(i,j));
      }
    }
    B_kernel_pub_.publish(B_kernel_msg);
  }

  Eigen::MatrixXd A_tor = Eigen::MatrixXd::Zero(mode_num_*2, mode_num_*2);
  Eigen::MatrixXd B_tor = Eigen::MatrixXd::Zero(mode_num_*2, motor_num_);
  Eigen::MatrixXd C_tor = Eigen::MatrixXd::Zero(mode_num_, mode_num_*2);

  if (!is_use_rbdl_torsion_B_) {
    Eigen::MatrixXd B_torsion_tmp = Eigen::MatrixXd::Zero(torsion_num_, motor_num_);
    for (int i = 0; i < torsion_num_; ++i) {
      for (int j = 0; j < motor_num_; ++j) {
        const KDL::Frame thrust_frame(robot_model_->getSegmentsTf().at("thrust"+std::to_string(j+1)));
        // const KDL::Frame link_frame(robot_model_->getSegmentsTf().at("link"+std::to_string(i+1)));
        const KDL::Frame link_frame(robot_model_->getSegmentsTf().at("thrust"+std::to_string(i+1)));
        KDL::Frame thrust_from_link = link_frame.Inverse() * thrust_frame;
        double moment_arm_length = thrust_from_link.p.y();
        B_torsion_tmp(i,j) = -moment_arm_length;
      }
    }
    torsion_B_matrix_ = torsion_mode_matrix_ * B_torsion_tmp;

    if (is_debug_ && debug_pub_throttle_count_%DEBUG_PUB_THROTTLE_FACTOR) {
      std_msgs::Float32MultiArray B_msg, B_mode_msg;
      B_msg.data.clear(); B_mode_msg.data.clear();
      for (int i = 0; i < B_torsion_tmp.rows(); ++i) {
        for (int j = 0; j < B_torsion_tmp.cols(); ++j) {
          B_msg.data.push_back(B_torsion_tmp(i,j));
        }
      }
      torsion_B_pub_.publish(B_msg);
      for (int i = 0; i < torsion_B_matrix_.rows(); ++i) {
        for (int j = 0; j < torsion_B_matrix_.cols(); ++j) {
          B_mode_msg.data.push_back(torsion_B_matrix_(i,j));
        }
      }
      torsion_B_mode_pub_.publish(B_mode_msg);
    }
  }

  for(int i = 0; i < mode_num_; i++) {
    A_tor(2*i, 2*i+1) = 1;
    A_tor(2*i+1, 2*i) = torsion_eigens_[i];
    B_tor.row(2*i+1) = torsion_B_matrix_.row(i);
    C_tor(i, 2*i) = 1;
  }

  A.block(0, 0, lqi_mode_*3, lqi_mode_*3) = A_eom;
  A.block(lqi_mode_*3, lqi_mode_*3, mode_num_*2, mode_num_*2) = A_tor;
  B.block(0,0,lqi_mode_*3, motor_num_) = B_eom;
  B.block(lqi_mode_*3, 0, mode_num_*2, motor_num_) = B_tor;
  C.block(0,0, lqi_mode_, lqi_mode_*3) = C_eom;
  C.block(lqi_mode_, lqi_mode_*3, mode_num_, mode_num_*2) = C_tor;

  ROS_DEBUG_STREAM("mode matrix: " << std::endl << torsion_mode_matrix_);
  ROS_DEBUG_STREAM("torsion B matrix: " << std::endl << torsion_B_matrix_);
  ROS_DEBUG_STREAM("torsion B rot matrix: " << std::endl << torsion_B_rot_matrix_);
  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: A: \n"  <<  A );
  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: B: \n"  <<  B );
  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: C: \n"  <<  C );

  Eigen::VectorXd q_diagonals(lqi_mode_*3+mode_num_*2);
  if(lqi_mode_ == 3)  {
    q_diagonals << lqi_z_weight_(0), lqi_z_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_z_weight_(1), lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1);
  } else {
    q_diagonals << lqi_z_weight_(0), lqi_z_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_yaw_weight_(0), lqi_yaw_weight_(2), lqi_z_weight_(1), lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1), lqi_yaw_weight_(1);
  }
  for (int i = 0; i < mode_num_; ++i) {
      q_diagonals(lqi_mode_*3+2*i) = q_mu_[i];
      q_diagonals(lqi_mode_*3+2*i+1) = q_mu_d_[i];
  }

  Eigen::MatrixXd Q = q_diagonals.asDiagonal();
  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: Q: \n"  <<  Q );

  Eigen::MatrixXd P_trans = P.topRows(3) / hydrus_robot_model_->getMass() ;
  Eigen::MatrixXd R_trans = P_trans.transpose() * P_trans;
  Eigen::MatrixXd R_input = Eigen::MatrixXd::Identity(motor_num_, motor_num_);
  Eigen::MatrixXd R = R_trans * trans_constraint_weight_ + R_input * att_control_weight_;

  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: R: \n"  <<  R );

  double t = ros::Time::now().toSec();

  if(K_.cols() != lqi_mode_ * 3) {
    resetGain(); // four axis -> three axis and vice versa
  }

  bool use_kleinman_method = true;
  if(K_.cols() == 0 || K_.rows() == 0) use_kleinman_method = false;
   if(!control_utils::care(A, B, R, Q, K_, use_kleinman_method)) {
    ROS_ERROR_STREAM("error in solver of continuous-time algebraic riccati equation");
    return false;
  }

  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator: CARE: %f sec" << ros::Time::now().toSec() - t);
  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator:  K \n" <<  K_);

  if (is_debug_ && debug_pub_throttle_count_%DEBUG_PUB_THROTTLE_FACTOR) {
    std_msgs::Float32MultiArray K_gain_msg;
    K_gain_msg.data.clear();
    for (int i = 0; i < K_.rows(); ++i) {
      for (int j = 0; j < K_.cols(); ++j) {
        K_gain_msg.data.push_back(K_(i,j));
      }
    }
    K_gain_pub_.publish(K_gain_msg);
  }

  // for torsion suppressive gains
  Eigen::VectorXd K_roll_torsion_corr = Eigen::VectorXd::Zero(mode_num_);
  Eigen::VectorXd K_pitch_torsion_corr = Eigen::VectorXd::Zero(mode_num_);
  Eigen::VectorXd K_yaw_torsion_corr = Eigen::VectorXd::Zero(mode_num_);

  // order is [roll_p, roll_i, roll_d, pitch_p, pitch_i, pitch_d, yaw_p, yaw_i, yaw_d]
  std::vector<double> K_torsion_factors(9, 1.0);
  Eigen::VectorXd K_factors_index(9);
  K_factors_index << 2,lqi_mode_*2+1,3,4,lqi_mode_*2+2,5,6,lqi_mode_*2+3,7;

  for (int i = 0; i < mode_num_; ++i) {
    double K_torsion_norm = K_.col(lqi_mode_*3+i*2).norm();
    ROS_DEBUG_STREAM("K torsion norm of mode " << i << " : " << K_torsion_norm);
    K_roll_torsion_corr[i] = K_.col(2).dot( K_.col(lqi_mode_*3+i*2) ) / K_.col(2).norm() / K_torsion_norm;
    K_pitch_torsion_corr[i] = K_.col(4).dot( K_.col(lqi_mode_*3+i*2) ) / K_.col(4).norm() / K_torsion_norm;
    K_yaw_torsion_corr[i] = K_.col(6).dot( K_.col(lqi_mode_*3+i*2) ) / K_.col(6).norm() / K_torsion_norm;

    for (int j = 0; j < 9; ++j) {
      double corr_squared = 0;
      if      (int(j/3) == X) { corr_squared = K_roll_torsion_corr(i)*K_roll_torsion_corr(i); }
      else if (int(j/3) == Y) { corr_squared = K_pitch_torsion_corr(i)*K_pitch_torsion_corr(i); }
      else if (int(j/3) == Z) { corr_squared = K_yaw_torsion_corr(i)*K_yaw_torsion_corr(i); }
      K_torsion_factors[j] -= torsion_alpha_[j] * K_torsion_norm * corr_squared * torsion_eigens_[0]/torsion_eigens_[i];
    }
  }
  ROS_DEBUG_STREAM("torsion corr roll :\n" << K_roll_torsion_corr);
  ROS_DEBUG_STREAM("torsion corr pitch:\n" << K_pitch_torsion_corr);
  ROS_DEBUG_STREAM("torsion corr yaw  :\n" << K_yaw_torsion_corr);
  ROS_DEBUG_STREAM("torsion factor roll : P: " << K_torsion_factors[0] << ", I: " << K_torsion_factors[1] << ", D: " << K_torsion_factors[2] );
  ROS_DEBUG_STREAM("torsion factor pitch: P: " << K_torsion_factors[3] << ", I: " << K_torsion_factors[4] << ", D: " << K_torsion_factors[5] );
  ROS_DEBUG_STREAM("torsion factor yaw  : P: " << K_torsion_factors[6] << ", I: " << K_torsion_factors[7] << ", D: " << K_torsion_factors[8] );

  // gain shift using null space
  if (is_use_torsion_null_space_shift_) {
    if (nlopt_throttle_count_%nlopt_throttle_factor_) {
      Eigen::MatrixXd K_gain_for_shift(motor_num_, 3);
      K_gain_for_shift.col(0) = K_.col(K_factors_index(0));
      K_gain_for_shift.col(1) = K_.col(K_factors_index(3));
      K_gain_for_shift.col(2) = K_.col(K_factors_index(6));
      EigenMatrixFloat32MultiArrayPublish(K_gain_shift_pub_, K_gain_for_shift);

      Eigen::MatrixXd K_mode(motor_num_, mode_num_);
      for (int i = 0; i < mode_num_; ++i) {
        K_mode.col(i) = K_.col(lqi_mode_*3+i*2);
      }
      //EigenMatrixFloat32MultiArrayPublish(K_mode_shift_pub_, K_mode); TODO ONLY FOR DEBUG torsion_mode_calculator!

      EigenMatrixFloat32MultiArrayPublish(B_eom_kernel_pub_, B_eom_kernel_);

      Eigen::MatrixXd nlopt_alpha(K_gain_for_shift.cols(), mode_num_);
      for (int i = 0; i < mode_num_; ++i) {
        nlopt_alpha(0, i) = torsion_alpha_.at(0) * torsion_eigens_.at(0)/torsion_eigens_.at(i);
        nlopt_alpha(1, i) = torsion_alpha_.at(3) * torsion_eigens_.at(0)/torsion_eigens_.at(i);
        nlopt_alpha(2, i) = torsion_alpha_.at(6) * torsion_eigens_.at(0)/torsion_eigens_.at(i);
      }
      EigenMatrixFloat32MultiArrayPublish(nlopt_alpha_pub_, nlopt_alpha);
    }
    nlopt_throttle_count_++;

    for (int i = 0; i < 9; ++i) {
      if (K_torsion_factors[i] < null_space_shift_thresh_) {
        // mix
        double limit_factor = 1.0;
        double max_ratio = std::max(std::abs(*std::max_element(kernel_mix_ratio_[i].begin(), kernel_mix_ratio_[i].end())), std::abs(*std::min_element(kernel_mix_ratio_[i].begin(), kernel_mix_ratio_[i].end())));
        double max_gain = K_.col(K_factors_index[i]).cwiseAbs().maxCoeff();
        if (max_ratio > null_space_shift_mix_limit_) {
          limit_factor = null_space_shift_mix_limit_ / max_ratio;
        }
        double norm_org = K_.col(K_factors_index[i]).norm();

        for (int j = 0; j < B_eom_kernel_.cols(); ++j) {
          K_.col(K_factors_index[i]) += kernel_mix_ratio_[i][j] * B_eom_kernel_.col(j) * norm_org * limit_factor;
        }
        K_.col(K_factors_index[i]) = K_.col(K_factors_index[i]) * norm_org/K_.col(K_factors_index[i]).norm();
        // K_.col(K_factors_index[i]) = K_.col(K_factors_index[i]) * max_gain/K_.col(K_factors_index[i]).cwiseAbs().maxCoeff();
        // K_torsion_factors[i] = max_f;
      }
    }
  }

  for (int i = 0; i < 9; ++i) {
    K_torsion_factors[i] = std::max(K_torsion_factors[i], torsion_epsilon_[i]);
  }

  for(int i = 0; i < motor_num_; ++i) {
    roll_gains_.at(i) = Eigen::Vector3d(-K_(i,2)*K_torsion_factors[0],  K_(i, lqi_mode_ * 2 + 1)*K_torsion_factors[1], -K_(i,3)*K_torsion_factors[2]);
    pitch_gains_.at(i) = Eigen::Vector3d(-K_(i,4)*K_torsion_factors[3], K_(i, lqi_mode_ * 2 + 2)*K_torsion_factors[4], -K_(i,5)*K_torsion_factors[5]);
    z_gains_.at(i) = Eigen::Vector3d(-K_(i,0), K_(i, lqi_mode_ * 2), -K_(i,1));
    if(lqi_mode_ == 4) {
      yaw_gains_.at(i) = Eigen::Vector3d(-K_(i,6)*K_torsion_factors[6], K_(i, lqi_mode_ * 2 + 3)*K_torsion_factors[7], -K_(i,7)*K_torsion_factors[8]);
    } else {
      yaw_gains_.at(i).setZero();
    }
  }

  for (int i = 0; i < mode_num_; ++i) {
    q_mu_p_gains_.at(i).clear();
    q_mu_d_gains_.at(i).clear();
    for (int j = 0; j < motor_num_; ++j) {
      q_mu_p_gains_[i].push_back(K_(j, lqi_mode_*3+i*2)); // *torsion_eigens_[0] / torsion_eigens_[i]);
      q_mu_d_gains_[i].push_back(K_(j, lqi_mode_*3+1+i*2)); // *torsion_eigens_[0] / torsion_eigens_[i]);
    }
  }

  // for debug
  if (is_debug_ && debug_pub_throttle_count_%DEBUG_PUB_THROTTLE_FACTOR) {
    std_msgs::Float32MultiArray K_gain_msg;
    K_gain_msg.data.clear();
    for (int i = 0; i < K_.rows(); ++i) {
      for (int j = 0; j < K_.cols(); ++j) {
        K_gain_msg.data.push_back(K_(i,j));
      }
    }
    K_gain_shifted_pub_.publish(K_gain_msg);
  }

  if (is_debug_) { debug_pub_throttle_count_++; }

  // compensation for gyro moment
  p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, lqi_mode_));
  return true;
}

void HydrusTiltedLQITorsionModeController::publishGain()
{
  HydrusLQIController::publishGain();

  double roll,pitch, yaw;
  robot_model_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

  spinal::DesireCoord coord_msg;
  coord_msg.roll = roll;
  coord_msg.pitch = pitch;
  desired_baselink_rot_pub_.publish(coord_msg);
}

void HydrusTiltedLQITorsionModeController::rosParamInit()
{
  HydrusLQIController::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");
  getParam<bool>(lqi_nh, "debug", is_debug_, false);

  getParam<bool>(lqi_nh, "use_rbdl_b_rot", use_rbdl_b_rot_, false);
  getParam<double>(lqi_nh, "trans_constraint_weight", trans_constraint_weight_, 1.0);
  getParam<double>(lqi_nh, "att_control_weight", att_control_weight_, 1.0);

  getParam<double>(lqi_nh, "torsional_spring_constant", torsional_spring_constant_, 1.0);
  getParam<int>(lqi_nh, "mode_num", mode_num_, motor_num_-4);
  getParam<double>(lqi_nh, "init_wait_time", init_wait_time_, 1.0);
  getParam<bool>(lqi_nh, "use_rbdl_torsion_b", is_use_rbdl_torsion_B_, true);

  lqi_nh.getParam("q_mu", q_mu_);
  lqi_nh.getParam("q_mu_d", q_mu_d_);

  ros::NodeHandle gain_shift_nh(control_nh, "gain_shift");
  getParam<bool>(lqi_nh, "use_torsion_null_space_shift", is_use_torsion_null_space_shift_, true);
  getParam<double>(gain_shift_nh, "null_space_shift_thresh", null_space_shift_thresh_, 0.7);
  getParam<double>(gain_shift_nh, "null_space_shift_mix_limit", null_space_shift_mix_limit_, 0.7);
  getParam<int>(lqi_nh, "nlopt_throttle_factor", nlopt_throttle_factor_, 1);
  double torsion_alpha_rp_p;
  double torsion_alpha_rp_i;
  double torsion_alpha_rp_d;
  double torsion_alpha_y_p;
  double torsion_alpha_y_i;
  double torsion_alpha_y_d;
  double torsion_epsilon_rp_p;
  double torsion_epsilon_rp_i;
  double torsion_epsilon_rp_d;
  double torsion_epsilon_y_p;
  double torsion_epsilon_y_i;
  double torsion_epsilon_y_d;
  getParam<double>(lqi_nh, "torsion_epsilon_rp_p", torsion_epsilon_rp_p, 1);
  getParam<double>(lqi_nh, "torsion_epsilon_rp_i", torsion_epsilon_rp_i, 1);
  getParam<double>(lqi_nh, "torsion_epsilon_rp_d", torsion_epsilon_rp_d, 1);
  getParam<double>(lqi_nh, "torsion_epsilon_y_p", torsion_epsilon_y_p, 1);
  getParam<double>(lqi_nh, "torsion_epsilon_y_i", torsion_epsilon_y_i, 1);
  getParam<double>(lqi_nh, "torsion_epsilon_y_d", torsion_epsilon_y_d, 1);
  getParam<double>(lqi_nh, "torsion_alpha_rp_p", torsion_alpha_rp_p, 0.0);
  getParam<double>(lqi_nh, "torsion_alpha_rp_i", torsion_alpha_rp_i, 0.0);
  getParam<double>(lqi_nh, "torsion_alpha_rp_d", torsion_alpha_rp_d, 0.0);
  getParam<double>(lqi_nh, "torsion_alpha_y_p", torsion_alpha_y_p, 0.0);
  getParam<double>(lqi_nh, "torsion_alpha_y_i", torsion_alpha_y_i, 0.0);
  getParam<double>(lqi_nh, "torsion_alpha_y_d", torsion_alpha_y_d, 0.0);
  torsion_epsilon_[0] = torsion_epsilon_rp_p;
  torsion_epsilon_[1] = torsion_epsilon_rp_i;
  torsion_epsilon_[2] = torsion_epsilon_rp_d;
  torsion_epsilon_[3] = torsion_epsilon_rp_p;
  torsion_epsilon_[4] = torsion_epsilon_rp_i;
  torsion_epsilon_[5] = torsion_epsilon_rp_d;
  torsion_epsilon_[6] = torsion_epsilon_y_p;
  torsion_epsilon_[7] = torsion_epsilon_y_i;
  torsion_epsilon_[8] = torsion_epsilon_y_d;
  torsion_alpha_[0] = torsion_alpha_rp_p;
  torsion_alpha_[1] = torsion_alpha_rp_i;
  torsion_alpha_[2] = torsion_alpha_rp_d;
  torsion_alpha_[3] = torsion_alpha_rp_p;
  torsion_alpha_[4] = torsion_alpha_rp_i;
  torsion_alpha_[5] = torsion_alpha_rp_d;
  torsion_alpha_[6] = torsion_alpha_y_p;
  torsion_alpha_[7] = torsion_alpha_y_i;
  torsion_alpha_[8] = torsion_alpha_y_d;
}

void HydrusTiltedLQITorsionModeController::cfgLQITorsionCallback(hydrus::LQI_torsionConfig &config, uint32_t level)
{
  double ratio;
  if(config.lqi_gain_flag)
  {
    printf("LQI Torsion Param:");
    switch(level)
    {
      case LQI_TORSION_MU_P_GAIN:
        ratio = config.q_mu / q_mu_[0];
        for (int i = 0; i < mode_num_; ++i) {
          q_mu_[i] = ratio * q_mu_[i];
        }
        printf("change the gain of lqi torsion mu p gain: %f\n", q_mu_[0]);
        break;
      case LQI_TORSION_MU_D_GAIN:
        ratio = config.q_mu_d / q_mu_d_[0];
        for (int i = 0; i < mode_num_; ++i) {
          q_mu_d_[i] = ratio * q_mu_d_[i];
        }
        printf("change the gain of lqi torsion mu d gain: %f\n", q_mu_d_[0]);
        break;
      case LQI_TORSION_EPSILON_RP_P:
        torsion_epsilon_[0] = config.torsion_epsilon_rp_p;
        torsion_epsilon_[3] = config.torsion_epsilon_rp_p;
        printf("change the gain of lqi torsion epsilon limit for roll&pitch p: %f\n", config.torsion_epsilon_rp_p);
        break;
      case LQI_TORSION_EPSILON_RP_I:
        torsion_epsilon_[1] = config.torsion_epsilon_rp_i;
        torsion_epsilon_[4] = config.torsion_epsilon_rp_i;
        printf("change the gain of lqi torsion epsilon limit for roll&pitch i: %f\n", config.torsion_epsilon_rp_i);
        break;
      case LQI_TORSION_EPSILON_RP_D:
        torsion_epsilon_[2] = config.torsion_epsilon_rp_d;
        torsion_epsilon_[5] = config.torsion_epsilon_rp_d;
        printf("change the gain of lqi torsion epsilon limit for roll&pitch d: %f\n", config.torsion_epsilon_rp_d);
        break;
      case LQI_TORSION_EPSILON_Y_P:
        torsion_epsilon_[6] = config.torsion_epsilon_y_p;
        printf("change the gain of lqi torsion epsilon limit for yaw p: %f\n", config.torsion_epsilon_y_p);
        break;
      case LQI_TORSION_EPSILON_Y_I:
        torsion_epsilon_[7] = config.torsion_epsilon_y_i;
        printf("change the gain of lqi torsion epsilon limit for yaw i: %f\n", config.torsion_epsilon_y_i);
        break;
      case LQI_TORSION_EPSILON_Y_D:
        torsion_epsilon_[8] = config.torsion_epsilon_y_d;
        printf("change the gain of lqi torsion epsilon limit for yaw d: %f\n", config.torsion_epsilon_y_d);
        break;
      case LQI_TORSION_ALPHA_RP_P:
        torsion_alpha_[0] = config.torsion_alpha_rp_p;
        torsion_alpha_[3] = config.torsion_alpha_rp_p;
        printf("change the gain of lqi torsion alpha const for roll&pitch p: %f\n", config.torsion_alpha_rp_p);
        break;
      case LQI_TORSION_ALPHA_RP_I:
        torsion_alpha_[1] = config.torsion_alpha_rp_i;
        torsion_alpha_[4] = config.torsion_alpha_rp_i;
        printf("change the gain of lqi torsion alpha const for roll&pitch i: %f\n", config.torsion_alpha_rp_i);
        break;
      case LQI_TORSION_ALPHA_RP_D:
        torsion_alpha_[2] = config.torsion_alpha_rp_d;
        torsion_alpha_[5] = config.torsion_alpha_rp_d;
        printf("change the gain of lqi torsion alpha const for roll&pitch d: %f\n", config.torsion_alpha_rp_d);
        break;
      case LQI_TORSION_ALPHA_Y_P:
        torsion_alpha_[6] = config.torsion_alpha_y_p;
        printf("change the gain of lqi torsion alpha const for yaw p: %f\n", config.torsion_alpha_y_p);
        break;
      case LQI_TORSION_ALPHA_Y_I:
        torsion_alpha_[7] = config.torsion_alpha_y_i;
        printf("change the gain of lqi torsion alpha const for yaw i: %f\n", config.torsion_alpha_y_i);
        break;
      case LQI_TORSION_ALPHA_Y_D:
        torsion_alpha_[8] = config.torsion_alpha_y_d;
        printf("change the gain of lqi torsion alpha const for yaw d: %f\n", config.torsion_alpha_y_d);
        break;
      default:
        printf("\n");
        break;
    }
  }
}

void HydrusTiltedLQITorsionModeController::linkTorsionCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  copy(msg->position.begin(), msg->position.end(), torsions_.begin());
  copy(msg->velocity.begin(), msg->velocity.end(), torsions_d_.begin());
}

void HydrusTiltedLQITorsionModeController::torsionEigensCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  torsion_eigens_.clear();
  for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
    torsion_eigens_.push_back(*it);
  }
}

void HydrusTiltedLQITorsionModeController::torsionModeCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  int i = 0;
  int j = 0;
  for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
    torsion_mode_matrix_(j, i) = *it;
    i++;
    if (i==torsion_num_) {
      i = 0;
      j++;
    }
    if (j==mode_num_) {
      break;
    }
  }
}

void HydrusTiltedLQITorsionModeController::torsionBCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  int i = 0;
  int j = 0;
  for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
    torsion_B_matrix_(j, i) = *it;
    i++;
    if (i==motor_num_) {
      i = 0;
      j++;
    }
    if (j==mode_num_) {
      break;
    }
  }
}

void HydrusTiltedLQITorsionModeController::torsionBRotCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  int i = 0;
  int j = 0;
  for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
    torsion_B_rot_matrix_(j, i) = *it;
    i++;
    if (i==motor_num_) {
      i = 0;
      j++;
    }
    if (j==3) {
      break;
    }
  }
}

bool HydrusTiltedLQITorsionModeController::setQMuCallback(aerial_robot_msgs::SetFloat64Array::Request &req, aerial_robot_msgs::SetFloat64Array::Response &res) {
  copy(req.data.begin(), req.data.end(), q_mu_.begin());
  res.success = true;
  return true;
}

bool HydrusTiltedLQITorsionModeController::setQMuDCallback(aerial_robot_msgs::SetFloat64Array::Request &req, aerial_robot_msgs::SetFloat64Array::Response &res) {
  copy(req.data.begin(), req.data.end(), q_mu_d_.begin());
  res.success = true;
  return true;
}

void HydrusTiltedLQITorsionModeController::kernelMixRatioCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  Eigen::MatrixXd mat(msg->layout.dim[0].size, msg->layout.dim[1].size);
  for(int i = 0; i<msg->layout.dim[0].stride; i++) {
    mat(int(i/msg->layout.dim[1].stride), i%msg->layout.dim[1].stride) = msg->data[i];
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < B_eom_kernel_.cols(); ++j) {
      kernel_mix_ratio_[3*i][j] = mat(i, j); // p
      kernel_mix_ratio_[3*i+1][j] = -mat(i, j); // i
      kernel_mix_ratio_[3*i+2][j] = mat(i, j); // d
    }
  }
}

void HydrusTiltedLQITorsionModeController::EigenMatrixFloat32MultiArrayPublish(const ros::Publisher& pub, const Eigen::MatrixXd& mat)
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.resize(mat.cols() * mat.rows());
  msg.layout.data_offset = 0;
  msg.layout.dim.resize(2);
  msg.layout.dim[0].label = "row";
  msg.layout.dim[0].size = mat.rows();
  msg.layout.dim[0].stride = mat.cols() * mat.rows();
  msg.layout.dim[1].label = "column";
  msg.layout.dim[1].size = mat.cols();
  msg.layout.dim[1].stride = mat.cols();

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      msg.data[i*msg.layout.dim[1].stride + j] = mat(i,j);
    }
  }

  pub.publish(msg);
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQITorsionModeController, aerial_robot_control::ControlBase);
