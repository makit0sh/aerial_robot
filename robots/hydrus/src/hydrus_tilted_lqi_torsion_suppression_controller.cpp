#include <hydrus/hydrus_tilted_lqi_torsion_suppression_controller.h>

using namespace aerial_robot_control;

void HydrusTiltedLQITorsionSuppressionController::initialize(ros::NodeHandle nh,
                                           ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  HydrusLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  desired_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);

  pid_msg_.z.p_term.resize(1);
  pid_msg_.z.i_term.resize(1);
  pid_msg_.z.d_term.resize(1);

  //rosParamInit();
  q_mu_p_gains_.resize(torsion_num_);
  q_mu_d_gains_.resize(torsion_num_);
  torsions_.resize(torsion_num_);
  torsions_d_.resize(torsion_num_);

  link_torsion_sub_ = nh_.subscribe<sensor_msgs::JointState>("link_torsion/joint_states", 1, &HydrusTiltedLQITorsionSuppressionController::linkTorsionCallback, this, ros::TransportHints().tcpNoDelay());

  //dynamic reconfigure server
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle cfgTorsion_nh(control_nh, "lqi_torsion");
  lqi_torsion_server_ = new dynamic_reconfigure::Server<hydrus::LQI_torsionConfig>(cfgTorsion_nh);
  dynamic_reconf_func_lqi_torsion_ = boost::bind(&HydrusTiltedLQITorsionSuppressionController::cfgLQITorsionCallback, this, _1, _2);
  lqi_torsion_server_->setCallback(dynamic_reconf_func_lqi_torsion_);
}

HydrusTiltedLQITorsionSuppressionController::~HydrusTiltedLQITorsionSuppressionController()
{
  delete lqi_torsion_server_;
}

void HydrusTiltedLQITorsionSuppressionController::controlCore()
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
  double psi_err = pid_controllers_.at(YAW).getErrP();
  std::vector<double> torsions_target_p(torsion_num_);
  std::vector<double> torsions_target_d(torsion_num_);
  for (int i = 0; i < torsion_num_; ++i) {
    // TODO improve
    torsions_target_p[i] = -yaw_torsion_p_gain_ * psi_err;
    torsions_target_d[i] = -yaw_torsion_d_gain_ * psi_err;
  }

  for (int i = 0; i < motor_num_; ++i) {
    for (int j = 0; j < torsion_num_; ++j) {
      target_base_thrust_[i] += q_mu_p_gains_[j][i] * (torsions_[j]-torsions_target_p[j]) + q_mu_d_gains_[j][i] * (torsions_d_[j]-torsions_target_d[j]);
    }
  }
}

bool HydrusTiltedLQITorsionSuppressionController::optimalGain()
{
  /* use z in calculation */
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(lqi_mode_, motor_num_);
  Eigen::MatrixXd inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  P_dash.row(0) = P.row(2) / robot_model_->getMass(); // z
  P_dash.bottomRows(lqi_mode_ - 1) = (inertia.inverse() * P.bottomRows(3)).topRows(lqi_mode_ - 1); // roll, pitch, yaw

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(lqi_mode_ * 3 +torsion_num_*2, lqi_mode_ * 3 +torsion_num_*2);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(lqi_mode_ * 3 +torsion_num_*2, motor_num_);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(lqi_mode_ +torsion_num_, lqi_mode_ * 3 +torsion_num_*2);

  Eigen::MatrixXd A_eom = Eigen::MatrixXd::Zero(lqi_mode_ * 3, lqi_mode_ * 3);
  Eigen::MatrixXd B_eom = Eigen::MatrixXd::Zero(lqi_mode_ * 3, motor_num_);
  Eigen::MatrixXd C_eom = Eigen::MatrixXd::Zero(lqi_mode_, lqi_mode_ * 3);
  for(int i = 0; i < lqi_mode_; i++)
    {
      A_eom(2 * i, 2 * i + 1) = 1;
      B_eom.row(2 * i + 1) = P_dash.row(i);
      C_eom(i, 2 * i) = 1;
    }
  A_eom.block(lqi_mode_ * 2, 0, lqi_mode_, lqi_mode_ * 3) = -C_eom;

  for (int i = 0; i < motor_num_; ++i) {
    //ROS_INFO_STREAM("yaw arm length of link " << i+1 <<" : " << getYawMomentArmLength(motor_num_/2, i+1));
    // no tilt だとjdid not work 全部負になって制御できなくなった
    // tilt 入れると動いたが、この項が効果あるかは不明、なくても動いた
    //ROS_INFO_STREAM("yaw b    at " << i+1 << " : " << B_eom(7, i));
    //ROS_INFO_STREAM("yaw btor at " << i+1 << " : " << -torsions_[0] * getYawMomentArmLength(motor_num_/2, i+1));
     B_eom(7, i) -= torsions_[0] * getYawMomentArmLength(motor_num_/2, i+1);;
  }

  Eigen::MatrixXd A_tor = Eigen::MatrixXd::Zero(torsion_num_*2, torsion_num_*2);
  Eigen::MatrixXd B_tor = Eigen::MatrixXd::Zero(torsion_num_*2, motor_num_);
  Eigen::MatrixXd C_tor = Eigen::MatrixXd::Zero(torsion_num_, torsion_num_*2);

  for(int i = 0; i < torsion_num_; i++) {
    double I1, I2;
    Eigen::VectorXd Q_mu = Eigen::VectorXd::Zero(motor_num_);
    KDL::RigidBodyInertia front_inertia_from_half_link;
    KDL::RigidBodyInertia back_inertia_from_half_link;
    KDL::Frame front_cog, back_cog;
    const double gravity_acc = 9.8;
    if (motor_num_ % 2 == 0) {
      front_inertia_from_half_link = getHalfRotInertiaAroundLink(motor_num_/2+1, true);
      back_inertia_from_half_link  = getHalfRotInertiaAroundLink(motor_num_/2, false);
      I1 = front_inertia_from_half_link.getRotationalInertia().data[0];
      I2 = back_inertia_from_half_link.getRotationalInertia().data[0];
      for (int j=0; j<motor_num_/2; j++) {
        Q_mu(j) = -getTorsionalMomentArmLength(motor_num_/2+1, j+1) /I1;
      }
      for (int j=motor_num_/2; j<motor_num_; j++) {
        Q_mu(j) = getTorsionalMomentArmLength(motor_num_/2, j+1) /I2;
      }
    } else {
      front_inertia_from_half_link = getHalfRotInertiaAroundLink((motor_num_+1)/2, true);
      back_inertia_from_half_link  = getHalfRotInertiaAroundLink((motor_num_+1)/2, false);
      I1 = front_inertia_from_half_link.getRotationalInertia().data[0];
      I2 = back_inertia_from_half_link.getRotationalInertia().data[0];
      for (int j=0; j<(motor_num_+1)/2; j++) {
        Q_mu(j) = -getTorsionalMomentArmLength((motor_num_+1)/2, j+1) /I1;
      }
      for (int j=(motor_num_+1)/2; j<motor_num_; j++) {
        Q_mu(j) = getTorsionalMomentArmLength((motor_num_+1)/2, j+1) /I2;
      }
    }
    A_tor(2*i, 2*i+1) = 1;
    A_tor(2*i+1, 2*i) = -(1/I1+1/I2) * torsional_spring_constant_;

    // yaw couppling TODO
    // double front_back_cog_dist = (front_inertia_from_half_link.getCOG() - back_inertia_from_half_link.getCOG()).Norm();
    // A(7, 8+i*2) += -hydrus_robot_model_->getMass() * gravity_acc /4 * front_back_cog_dist / hydrus_robot_model_->getInertia<KDL::RotationalInertia>().data[9];

    B_tor.row(2*i+1) = Q_mu;
    C_tor(i, 2*i) = 1;
  }

  A.block(0, 0, lqi_mode_*3, lqi_mode_*3) = A_eom;
  A.block(lqi_mode_*3, lqi_mode_*3, torsion_num_*2, torsion_num_*2) = A_tor;
  B.block(0,0,lqi_mode_*3, motor_num_) = B_eom;
  B.block(lqi_mode_*3, 0, torsion_num_*2, motor_num_) = B_tor;
  C.block(0,0, lqi_mode_, lqi_mode_*3) = C_eom;
  C.block(lqi_mode_, lqi_mode_*3, torsion_num_, torsion_num_*2) = C_tor;

  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: A: \n"  <<  A );
  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: B: \n"  <<  B );
  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: C: \n"  <<  C );

  Eigen::VectorXd q_diagonals(lqi_mode_*3+torsion_num_*2);
  if(lqi_mode_ == 3)  {
    q_diagonals << lqi_z_weight_(0), lqi_z_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_z_weight_(1), lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1);
  } else {
    q_diagonals << lqi_z_weight_(0), lqi_z_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_yaw_weight_(0), lqi_yaw_weight_(2), lqi_z_weight_(1), lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1), lqi_yaw_weight_(1);
  }
  for (int i = 0; i < torsion_num_; ++i) {
    q_diagonals(lqi_mode_*3+2*i) = q_mu_;
    q_diagonals(lqi_mode_*3+2*i+1) = q_mu_d_;
  }

  Eigen::MatrixXd Q = q_diagonals.asDiagonal();
  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: Q: \n"  <<  Q );

  // Eigen::MatrixXd R  = Eigen::MatrixXd::Zero(motor_num_, motor_num_);
  // for(int i = 0; i < motor_num_; ++i) R(i,i) = r_.at(i);
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
  //ROS_INFO_STREAM_THROTTLE_NAMED(2, "LQI gain generator",  "LQI gain generator:  K \n" <<  K_);

  for(int i = 0; i < motor_num_; ++i) {
    roll_gains_.at(i) = Eigen::Vector3d(-K_(i,2),  K_(i, lqi_mode_ * 2 + 1), -K_(i,3));
    pitch_gains_.at(i) = Eigen::Vector3d(-K_(i,4), K_(i, lqi_mode_ * 2 + 2), -K_(i,5));
    z_gains_.at(i) = Eigen::Vector3d(-K_(i,0), K_(i, lqi_mode_ * 2), -K_(i,1));
    if(lqi_mode_ == 4) yaw_gains_.at(i) = Eigen::Vector3d(-K_(i,6), K_(i, lqi_mode_ * 2 + 3), -K_(i,7));
    else yaw_gains_.at(i).setZero();
  }

  for (int i = 0; i < torsion_num_; ++i) {
    q_mu_p_gains_.at(i).clear();
    q_mu_d_gains_.at(i).clear();
    for (int j = 0; j < motor_num_; ++j) {
      q_mu_p_gains_[i].push_back(K_(j, lqi_mode_*3+i*2));
      q_mu_d_gains_[i].push_back(K_(j, lqi_mode_*3+1+i*2));
    }
  }

  // compensation for gyro moment
  p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, lqi_mode_));
  return true;

/* no use of z in state, ces could not be solved WHY?????
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::MatrixXd P_dash  = inertia.inverse() * P.bottomRows(3); // roll, pitch, yaw
  ROS_INFO_STREAM("P: " << std::endl << P);

  Eigen::MatrixXd A_eom = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd B_eom = Eigen::MatrixXd::Zero(9, motor_num_);
  Eigen::MatrixXd C_eom = Eigen::MatrixXd::Zero(3, 9);
  for(int i = 0; i < 3; i++)
    {
      A_eom(2 * i, 2 * i + 1) = 1;
      B_eom.row(2 * i + 1) = P_dash.row(i);
      C_eom(i, 2 * i) = 1;
    }
  A_eom.block(6, 0, 3, 9) = -C_eom;

  Eigen::MatrixXd A_tor = Eigen::MatrixXd::Zero(torsion_num_*2, torsion_num_*2);
  Eigen::MatrixXd B_tor = Eigen::MatrixXd::Zero(torsion_num_*2, motor_num_);
  Eigen::MatrixXd C_tor = Eigen::MatrixXd::Zero(torsion_num_, torsion_num_*2);

  for(int i = 0; i < torsion_num_; i++) {
    double I1, I2;
    Eigen::VectorXd Q_mu = Eigen::VectorXd::Zero(motor_num_);
    KDL::RigidBodyInertia front_inertia_from_half_link;
    KDL::RigidBodyInertia back_inertia_from_half_link;
    KDL::Vector front_cog_, back_cog_;
    const double gravity_acc = 9.8;
    // TODO revise for tilted rotors
    if (motor_num_ % 2 == 0) {
      front_inertia_from_half_link = getHalfRotInertiaAroundLink(motor_num_/2+1, true);
      back_inertia_from_half_link  = getHalfRotInertiaAroundLink(motor_num_/2, false);
      I1 = front_inertia_from_half_link.getRotationalInertia().data[0];
      I2 = back_inertia_from_half_link.getRotationalInertia().data[0];
      for (int j=0; j<motor_num_/2; j++) {
        Q_mu(j) = -getTorsionalMomentArmLength(motor_num_/2+1, j+1) /I1;
      }
      for (int j=motor_num_/2; j<motor_num_; j++) {
        Q_mu(j) = getTorsionalMomentArmLength(motor_num_/2, j+1) /I2;
      }
    } else {
      front_inertia_from_half_link = getHalfRotInertiaAroundLink((motor_num_+1)/2, true);
      back_inertia_from_half_link  = getHalfRotInertiaAroundLink((motor_num_+1)/2, false);
      I1 = front_inertia_from_half_link.getRotationalInertia().data[0];
      I2 = back_inertia_from_half_link.getRotationalInertia().data[0];
      for (int j=0; j<(motor_num_+1)/2; j++) {
        Q_mu(j) = -getTorsionalMomentArmLength((motor_num_+1)/2, j+1) /I1;
      }
      for (int j=(motor_num_+1)/2; j<motor_num_; j++) {
        Q_mu(j) = getTorsionalMomentArmLength((motor_num_+1)/2, j+1) /I2;
      }
    }
    A_tor(2*i, 2*i+1) = 1;
    A_tor(2*i+1, 2*i) = -(1/I1+1/I2) * torsional_spring_constant_;

    B_tor.row(2*i+1) = Q_mu;
    C_tor(i, 2*i) = 1;
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9+torsion_num_*2, 9+torsion_num_*2);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9+torsion_num_*2, motor_num_);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3 +torsion_num_, 9 +torsion_num_*2);

  A.block(0, 0, 9, 9) = A_eom;
  A.block(9, 9, torsion_num_*2, torsion_num_*2) = A_tor;
  B.block(0,0,9, motor_num_) = B_eom;
  B.block(9, 0, torsion_num_*2, motor_num_) = B_tor;
  C.block(0,0, 3, 9) = C_eom;
  C.block(3, 9, 9, torsion_num_*2) = C_tor;

  //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: A: \n"  <<  A );
  //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: B: \n"  <<  B );
  //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: C: \n"  <<  C );

  Eigen::VectorXd q_diagonals(9+torsion_num_*2);
  q_diagonals << lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_yaw_weight_(0), lqi_yaw_weight_(2), lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1), lqi_yaw_weight_(1);

  for (int i = 0; i < torsion_num_; ++i) {
    q_diagonals(9+2*i) = q_mu_;
    q_diagonals(9+2*i+1) = q_mu_d_;
  }

  Eigen::MatrixXd Q = q_diagonals.asDiagonal();

  Eigen::MatrixXd P_trans = P.topRows(3) / hydrus_robot_model_->getMass() ;
  Eigen::MatrixXd R_trans = P_trans.transpose() * P_trans;
  Eigen::MatrixXd R_input = Eigen::MatrixXd::Identity(motor_num_, motor_num_);
  Eigen::MatrixXd R = R_trans * trans_constraint_weight_ + R_input * att_control_weight_;

  // There is BUG??? in care.cpp:125
  // double t = ros::Time::now().toSec();
  // bool use_kleinman_method = true;
  // if(K_.cols() == 0 || K_.rows() == 0) use_kleinman_method = false;
  // if(!control_utils::care(A, B, R, Q, K_, use_kleinman_method))
  //   {
  //     ROS_ERROR_STREAM("error in solver of continuous-time algebraic riccati equation");
  //     return false;
  //   }

  // ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator: CARE: %f sec" << ros::Time::now().toSec() - t);
  // ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator:  K \n" <<  K_);
  // temporaly use former method
  Eigen::MatrixXd R_inv = R.inverse();
  const int H_size_half = 9+torsion_num_*2;
  Eigen::MatrixXcd H = Eigen::MatrixXcd::Zero(H_size_half * 2, H_size_half * 2);
  H.block(0,0, H_size_half, H_size_half) = A.cast<std::complex<double> >();
  H.block(H_size_half, 0, H_size_half, H_size_half) = -(Q.cast<std::complex<double> >());
  H.block(0, H_size_half, H_size_half, H_size_half) = - (B * R_inv * B.transpose()).cast<std::complex<double> >();
  H.block(H_size_half, H_size_half, H_size_half, H_size_half) = - (A.transpose()).cast<std::complex<double> >();

  ROS_INFO_STREAM("ok1");
  //eigen solving
  ros::Time start_time = ros::Time::now();
  Eigen::ComplexEigenSolver<Eigen::MatrixXcd> ces;
  ces.compute(H);
  ROS_INFO_STREAM("ok2");

  Eigen::MatrixXcd phy = Eigen::MatrixXcd::Zero(H_size_half * 2, H_size_half);
  int j = 0;

  // TODO bug in range????
  //for(int i = 0; i < H_size_half * 2; i++) {
  for(int i = 0; i < ces.eigenvalues().size(); i++) {
    if(ces.eigenvalues()[i].real() < 0) {
      if(j >= H_size_half) {
        ROS_ERROR("nagativa sigular amount is larger");
        return false;
      }
      phy.col(j) = ces.eigenvectors().col(i);
      j++;
    }
  }
  if(j != H_size_half) {
    ROS_ERROR("nagativa sigular value amount is not enough");
    return false;
  }
  ROS_INFO_STREAM("ok3");
  Eigen::MatrixXcd f = phy.block(0, 0, H_size_half, H_size_half);
  Eigen::MatrixXcd g = phy.block(H_size_half, 0, H_size_half, H_size_half);
  Eigen::MatrixXcd f_inv  = f.inverse();
  Eigen::MatrixXcd P_tmp = g * f_inv;

  K_ = -R_inv * B.transpose() * P_tmp.real();
  ROS_INFO_STREAM_NAMED("LQI gain generator",  "LQI gain generator:  K \n" <<  K_);
  // TODO end of temporal method

  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i) = Eigen::Vector3d(-K_(i,0), K_(i,6), -K_(i,1));
      pitch_gains_.at(i) = Eigen::Vector3d(-K_(i,2),  K_(i,7), -K_(i,3));
      yaw_gains_.at(i) = Eigen::Vector3d(-K_(i,4), K_(i,8), -K_(i,5));
    }

  for (int i = 0; i < torsion_num_; ++i) {
    q_mu_p_gains_.at(i).clear();
    q_mu_d_gains_.at(i).clear();
    for (int j = 0; j < motor_num_; ++j) {
      q_mu_p_gains_[i].push_back(K_(j, 9+i*2));
      q_mu_d_gains_[i].push_back(K_(j, 10+i*2));
    }
  }

  // compensation for gyro moment
  p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, 4));
  return true;
*/
}

void HydrusTiltedLQITorsionSuppressionController::publishGain()
{
  HydrusLQIController::publishGain();

  double roll,pitch, yaw;
  robot_model_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

  spinal::DesireCoord coord_msg;
  coord_msg.roll = roll;
  coord_msg.pitch = pitch;
  desired_baselink_rot_pub_.publish(coord_msg);
}

void HydrusTiltedLQITorsionSuppressionController::rosParamInit()
{
  HydrusLQIController::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");

  getParam<double>(lqi_nh, "trans_constraint_weight", trans_constraint_weight_, 1.0);
  getParam<double>(lqi_nh, "att_control_weight", att_control_weight_, 1.0);

  getParam<double>(lqi_nh, "torsional_spring_constant", torsional_spring_constant_, 1.0);
  getParam<int>(lqi_nh, "torsion_num", torsion_num_, 1);
  getParam<double>(lqi_nh, "q_mu", q_mu_, 0.1);
  getParam<double>(lqi_nh, "q_mu_d", q_mu_d_, 0.1);
  getParam<double>(lqi_nh, "yaw_torsion_p_gain", yaw_torsion_p_gain_, 0.1);
  getParam<double>(lqi_nh, "yaw_torsion_d_gain", yaw_torsion_d_gain_, 0.1);
}

void HydrusTiltedLQITorsionSuppressionController::cfgLQITorsionCallback(hydrus::LQI_torsionConfig &config, uint32_t level)
{
  if(config.lqi_gain_flag)
  {
    printf("LQI Torsion Param:");
    switch(level)
    {
      case LQI_TORSION_MU_P_GAIN:
        q_mu_ = config.q_mu;
        printf("change the gain of lqi torsion mu p gain: %f\n", q_mu_);
        break;
      case LQI_TORSION_MU_D_GAIN:
        q_mu_d_= config.q_mu_d;
        printf("change the gain of lqi torsion mu d gain: %f\n", q_mu_d_);
        break;
      case LQI_TORSION_TARGET_YAW_P_GAIN:
        yaw_torsion_p_gain_ = config.yaw_torsion_p_gain;
        printf("change the gain of yaw to torsion fw p gain: %f\n", yaw_torsion_p_gain_);
        break;
      case LQI_TORSION_TARGET_YAW_D_GAIN:
        yaw_torsion_d_gain_ = config.yaw_torsion_d_gain;
        printf("change the gain of yaw to torsion fw d gain: %f\n", yaw_torsion_d_gain_);
        break;
      default :
        printf("\n");
        break;
    }
  }
}

void HydrusTiltedLQITorsionSuppressionController::linkTorsionCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  if (torsion_num_<=0) return;
  // TODO only mode 1 considered
  int torsion_sz = msg->position.size();
  torsions_[0] = 0; torsions_d_[0] = 0;
  for (int i = 0; i < torsion_sz; ++i) {
    torsions_[0] += (i>torsion_sz/2 ? -1 : 1) * msg->position[i];
    torsions_d_[0] += (i>torsion_sz/2 ? -1 : 1) * msg->velocity[i];
  }
  // copy(msg->position.begin(), msg->position.end(), torsions_.begin());
  // copy(msg->velocity.begin(), msg->velocity.end(), torsions_d_.begin());
}

KDL::RigidBodyInertia HydrusTiltedLQITorsionSuppressionController::getHalfRotInertiaAroundLink(int axis_link_idx, bool is_front)
{
  int start_link_idx=1;
  int end_link_idx=motor_num_;
  if (is_front == true) {
    end_link_idx=axis_link_idx-1;
  } else {
    start_link_idx = axis_link_idx+1;
  }
  const std::string axis_link_name = "link"+std::to_string(axis_link_idx);
  const KDL::Frame axis_link_f{robot_model_->getSegmentsTf().at(axis_link_name)};

  KDL::RigidBodyInertia result_inertia = KDL::RigidBodyInertia::Zero();
  for (int i = start_link_idx; i <= end_link_idx; ++i) {
    const KDL::Frame link_f(robot_model_->getSegmentsTf().at("link"+std::to_string(i)));
    const KDL::RigidBodyInertia link_inertia(robot_model_->getInertiaMap().at("link"+std::to_string(i)));
    KDL::RigidBodyInertia link_inertia_from_axis = link_f * link_inertia;
    result_inertia = result_inertia + link_inertia_from_axis;
  }
  return axis_link_f.Inverse() * result_inertia;
}

double HydrusTiltedLQITorsionSuppressionController::getTorsionalMomentArmLength(int axis_link_idx, int motor_idx)
{
  std::pair<int, int> key=std::make_pair(motor_idx, axis_link_idx);
  double result=0;
  const KDL::Frame thrust_frame(robot_model_->getSegmentsTf().at("thrust"+std::to_string(motor_idx)));
  const KDL::Frame link_frame(robot_model_->getSegmentsTf().at("link"+std::to_string(axis_link_idx)));
  KDL::Vector x_unit(1.0, 0.0, 0.0);
  KDL::Frame thrust_from_link = link_frame.Inverse() * thrust_frame;
  double projection2link_length = KDL::dot( thrust_from_link.p , x_unit);
  result = sqrt(thrust_from_link.p.Norm()*thrust_from_link.p.Norm() - projection2link_length*projection2link_length);
  return result;
}

double HydrusTiltedLQITorsionSuppressionController::getYawMomentArmLength(int axis_link_idx, int motor_idx)
{
  double result;
  const KDL::Frame thrust_frame_root(robot_model_->getSegmentsTf().at("thrust"+std::to_string(motor_idx)));
  const KDL::Frame link_frame(robot_model_->getSegmentsTf().at("link"+std::to_string(axis_link_idx)));
  const KDL::Frame cog_root(robot_model_->getCog<KDL::Frame>());
  KDL::Frame thrust_frame = link_frame.Inverse() * thrust_frame_root;
  KDL::Frame cog = link_frame.Inverse() * cog_root;
  KDL::Vector x_unit(1.0, 0.0, 0.0);
  KDL::Vector cog2thrust_from_link = thrust_frame.p - cog.p;
  result = abs( KDL::dot( cog2thrust_from_link, x_unit) );
  return result;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQITorsionSuppressionController, aerial_robot_control::ControlBase);
