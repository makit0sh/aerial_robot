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

  link_torsion_sub_ = nh_.subscribe<sensor_msgs::JointState>("link_torsion/joint_states", 10, &HydrusTiltedLQITorsionModeController::linkTorsionCallback, this);
  eigen_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("torsion_eigens", 10, &HydrusTiltedLQITorsionModeController::torsionEigensCallback, this);
  mode_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("torsion_mode_matrix", 10, &HydrusTiltedLQITorsionModeController::torsionModeCallback, this);
  B_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("torsion_B_mode_matrix", 10, &HydrusTiltedLQITorsionModeController::torsionBCallback, this);
  B_rot_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("torsion_B_rot_matrix", 10, &HydrusTiltedLQITorsionModeController::torsionBRotCallback, this);

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

  Eigen::MatrixXd A_tor = Eigen::MatrixXd::Zero(mode_num_*2, mode_num_*2);
  Eigen::MatrixXd B_tor = Eigen::MatrixXd::Zero(mode_num_*2, motor_num_);
  Eigen::MatrixXd C_tor = Eigen::MatrixXd::Zero(mode_num_, mode_num_*2);

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

  for(int i = 0; i < motor_num_; ++i) {
    roll_gains_.at(i) = Eigen::Vector3d(-K_(i,2),  K_(i, lqi_mode_ * 2 + 1), -K_(i,3));
    pitch_gains_.at(i) = Eigen::Vector3d(-K_(i,4), K_(i, lqi_mode_ * 2 + 2), -K_(i,5));
    z_gains_.at(i) = Eigen::Vector3d(-K_(i,0), K_(i, lqi_mode_ * 2), -K_(i,1));
    if(lqi_mode_ == 4) yaw_gains_.at(i) = Eigen::Vector3d(-K_(i,6), K_(i, lqi_mode_ * 2 + 3), -K_(i,7));
    else yaw_gains_.at(i).setZero();
  }

  for (int i = 0; i < mode_num_; ++i) {
    q_mu_p_gains_.at(i).clear();
    q_mu_d_gains_.at(i).clear();
    for (int j = 0; j < motor_num_; ++j) {
      q_mu_p_gains_[i].push_back(K_(j, lqi_mode_*3+i*2)); // *torsion_eigens_[0] / torsion_eigens_[i]);
      q_mu_d_gains_[i].push_back(K_(j, lqi_mode_*3+1+i*2)); // *torsion_eigens_[0] / torsion_eigens_[i]);
    }
  }

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

  getParam<bool>(lqi_nh, "use_rbdl_b_rot", use_rbdl_b_rot_, 1.0);
  getParam<double>(lqi_nh, "trans_constraint_weight", trans_constraint_weight_, 1.0);
  getParam<double>(lqi_nh, "att_control_weight", att_control_weight_, 1.0);

  getParam<double>(lqi_nh, "torsional_spring_constant", torsional_spring_constant_, 1.0);
  getParam<int>(lqi_nh, "mode_num", mode_num_, motor_num_-4);
  getParam<double>(lqi_nh, "init_wait_time", init_wait_time_, 1.0);

  lqi_nh.getParam("q_mu", q_mu_);
  lqi_nh.getParam("q_mu_d", q_mu_d_);
}

void HydrusTiltedLQITorsionModeController::cfgLQITorsionCallback(hydrus::LQI_torsionConfig &config, uint32_t level)
{
  if(config.lqi_gain_flag)
  {
    printf("LQI Torsion Param:");
    switch(level)
    {
      // TODO!!!!
      case LQI_TORSION_MU_P_GAIN:
        //q_mu_[0] = config.q_mu;
        //printf("change the gain of lqi torsion mu p gain: %f\n", q_mu_[0]);
        break;
      case LQI_TORSION_MU_D_GAIN:
        //q_mu_d_[0] = config.q_mu_d;
        //printf("change the gain of lqi torsion mu d gain: %f\n", q_mu_d_[0]);
        break;
      default :
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

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQITorsionModeController, aerial_robot_control::ControlBase);
