#include <hydrus/torsion_estimator.h>

TorsionEstimator::TorsionEstimator (ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  tfListener_(tfBuffer_)
{
  nh_private_.param("debug", debug_, false);
  nh_private_.param("simulation", is_simulation_, false);

  // parameters for system
  nh_private_.param("links", link_num_, 6);
  neuron_imu_data_.resize(link_num_);
  link_transforms_.resize(link_num_);
  R_ci_cache_.resize(link_num_-1);
  prev_torsions_.resize(link_num_-1);
  torsion_vel_in_prev1_.resize(link_num_-1);
  torsion_vel_in_prev2_.resize(link_num_-1);
  torsion_vel_out_prev1_.resize(link_num_-1);
  torsion_vel_out_prev2_.resize(link_num_-1);
  torsion_in_prev1_.resize(link_num_-1);
  torsion_in_prev2_.resize(link_num_-1);
  torsion_out_prev1_.resize(link_num_-1);
  torsion_out_prev2_.resize(link_num_-1);
  nh_private_.param<std::string>("robot_name", robot_name_, "hydrus");

  nh_private_.param("use_mocap", is_use_mocap_, true);
  nh_private_.param("use_ukf", is_use_ukf_, true);
  nh_private_.param("use_ekf", is_use_ekf_, false);
  nh_private_.param("use_orientation", is_use_orientation_, false);
  nh_private_.param("use_base_imu", is_use_base_imu_, true);
  nh_private_.param("use_external_joint_estimation", is_use_ext_joint_, true);
  nh_private_.param("use_adj_joint_calculation", is_use_adj_joint_calculation_, false);
  nh_private_.param("use_simple_system_model", is_use_simple_system_model_, false);
  nh_private_.param("use_K_sys", is_use_K_sys_, false);
  nh_private_.param("base_link_id", base_link_id_, int(link_num_/2));
  nh_private_.param("torsion_vel_cutoff_freq", torsion_vel_cutoff_freq_, 10.0);
  nh_private_.param("torsion_vel_q", torsion_vel_q_, 1.0);
  nh_private_.param("torsion_cutoff_freq", torsion_cutoff_freq_, 10.0);
  nh_private_.param("torsion_q", torsion_q_, 1.0);

  // robot model
  nh_private_.param("torsion_constant", torsion_constant_, 1.0);

  std::string control_model_urdf;
  nh_private_.getParam("robot_description_control", control_model_urdf);

  model_ = new RigidBodyDynamics::Model();
  // construct model from urdf
  if (!RigidBodyDynamics::Addons::URDFReadFromString (control_model_urdf.c_str(), model_, true)) {
    ROS_ERROR_STREAM("Error loading model " << control_model_urdf);
    abort();
  }
  ROS_DEBUG_STREAM("Degree of freedom overview:" << std::endl << RigidBodyDynamics::Utils::GetModelDOFOverview(*model_));
  ROS_DEBUG_STREAM("model hierarchy overview:" << std::endl << RigidBodyDynamics::Utils::GetModelHierarchy(*model_));
  /* TODO */
  torsion_dof_update_order_ = rbdl_util::get_torsion_dof_update_order(model_, link_num_-1);
  joint_dof_update_order_ = rbdl_util::get_torsion_dof_update_order(model_, link_num_-1, "link", 2);
  /* torsion_dof_update_order_ = std::vector<unsigned int>{9, 7, 10, 12, 14};  //TODO remove, only for debug */
  /* joint_dof_update_order_ = std::vector<unsigned int>{6, 8, 11, 13, 15};  //TODO remove, only for debug */

  Q_cache_ = RigidBodyDynamics::Math::VectorNd::Zero(model_->q_size);
  QDot_cache_ = RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);
  QDDot_cache_ = RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);
  Tau_cache_ = RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);

  // ros subscribers
  if (!is_use_mocap_) {
    nh_private_.param<std::string>("neuron_imu_frame_name_prefix",neuron_imu_frame_name_prefix_, "neuron");
    nh_private_.param<std::string>("neuron_imu_frame_name_suffix",neuron_imu_frame_name_suffix_, "_imu");
  } else {
    nh_private_.param<std::string>("mocap_frame_name_prefix",neuron_imu_frame_name_prefix_, "mocap/link");
    nh_private_.param<std::string>("mocap_frame_name_suffix",neuron_imu_frame_name_suffix_, "/pose");
  }
  for (int i = 0; i < link_num_; ++i) {
    if (!is_use_mocap_) {
      neuron_imu_subs_.push_back(nh_.subscribe<sensor_msgs::Imu>(
          neuron_imu_frame_name_prefix_+std::to_string(i+1)+neuron_imu_frame_name_suffix_,
          1, boost::bind(&TorsionEstimator::neuronIMUCallback, this, _1, i)));
    } else {
      if (is_simulation_) {
        neuron_imu_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
            neuron_imu_frame_name_prefix_+std::to_string(i+1)+neuron_imu_frame_name_suffix_,
            1, boost::bind(&TorsionEstimator::simMocapCallback, this, _1, i)));
      } else {
        neuron_imu_subs_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(
            neuron_imu_frame_name_prefix_+std::to_string(i+1)+neuron_imu_frame_name_suffix_,
            1, boost::bind(&TorsionEstimator::mocapCallback, this, _1, i)));
      }
    }
    neuron_imu_data_[i].orientation.x = 0;
    neuron_imu_data_[i].orientation.y = 0;
    neuron_imu_data_[i].orientation.z = 0;
    neuron_imu_data_[i].orientation.w = 1;
  }

  joints_.resize(link_num_-1);
  joints_d_.resize(link_num_-1);
  jointstate_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, boost::bind(&TorsionEstimator::jointsCallback, this, _1));
  if (is_use_ext_joint_) {
    torsions_.resize(link_num_-1);
    torsionstate_sub_ = nh_.subscribe<sensor_msgs::JointState>("link_torsion/joint_states", 1, boost::bind(&TorsionEstimator::torsionCallback, this, _1));
  }

  base_angles_.resize(3);
  base_gyro_.resize(3);
  if (is_use_base_imu_) {
    baseImu_sub_ = nh_.subscribe<spinal::Imu>("imu", 1, boost::bind(&TorsionEstimator::baseImuCallback, this, _1));
  }

  // initialize kalman filter
  state_size_ = 2*(link_num_-1);
  control_size_ = 6+3*link_num_; // baselink attitude & joint angles and their deriv TODO include thrust
  if (is_use_ext_joint_) {
    measure_size_ = 7*(link_num_-1);
  } else {
    measure_size_ = 6*(link_num_-1);
  }
  if (is_use_ukf_) {
    ukf_ = new kf::UnscentedKalmanFilter(control_size_,measure_size_,state_size_);
    ukf_->setProcessFunction(boost::bind(&TorsionEstimator::processFunction, this, _1, _2));
    ukf_->setOutputFunction(boost::bind(&TorsionEstimator::outputFunction, this, _1));
  } else if (is_use_ekf_) {
    ekf_ = new kf::ExtendedKalmanFilter(control_size_,measure_size_,state_size_);
    ekf_->setProcessFunction(boost::bind(&TorsionEstimator::processFunction, this, _1, _2));
    ekf_->setOutputFunction(boost::bind(&TorsionEstimator::outputFunction, this, _1));
    ekf_->setProcessJacobian(boost::bind(&TorsionEstimator::processJacobian, this, _1, _2));
    ekf_->setOutputJacobian(boost::bind(&TorsionEstimator::outputJacobian, this, _1));
  }

  if (is_use_ukf_) {
    double ukf_alpha, ukf_beta, ukf_kappa;
    nh_private_.param("ukf_alpha", ukf_alpha, 0.05);
    nh_private_.param("ukf_beta", ukf_beta, 2.0);
    nh_private_.param("ukf_kappa", ukf_kappa, 0.0);
    ukf_->setDesignParameters(ukf_alpha, ukf_beta, ukf_kappa);
  }

  std::vector<double> sigma2_system, sigma2_measure;
  nh_private_.getParam("sigma2_system", sigma2_system);
  nh_private_.getParam("sigma2_measure", sigma2_measure);
  arma::mat kf_Q, kf_R;
  kf_Q = arma::diagmat(arma::vec(sigma2_system));
  kf_R = arma::diagmat(arma::vec(sigma2_measure));
  if (is_use_ukf_) {
    ukf_->setProcessCovariance(kf_Q);
    ukf_->setOutputCovariance(kf_R);
  } else if (is_use_ekf_) {
    ekf_->setProcessCovariance(kf_Q);
    ekf_->setOutputCovariance(kf_R);
  }


  if (is_use_simple_system_model_ && is_use_K_sys_) {
    torsion_K_sys_matrix_.resize(model_->q_size, model_->q_size);
    K_sys_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("torsion_K_sys_matrix", 10, &TorsionEstimator::torsionKsysCallback, this);
  }

  // ros publisher
  estimate_state_pub_ = nh_private_.advertise<sensor_msgs::JointState>("torsion_estimate", 5);

  // ros timer
  kf_prev_step_time_ = ros::Time::now();
  nh_private_.param("kf_step_rate", kf_step_rate_, 30.0);
  kf_timer_ = nh.createTimer(ros::Duration(1/kf_step_rate_), boost::bind(&TorsionEstimator::kfStepCallback, this, _1));
}

TorsionEstimator::~TorsionEstimator()
{
  if (is_use_ukf_) {
    delete ukf_;
  } else if (is_use_ekf_) {
    delete ekf_;
  }
}

void TorsionEstimator::kfStepCallback(const ros::TimerEvent& e)
{
  arma::vec u_control(control_size_);
  arma::vec y_measure(measure_size_);

  if (is_use_adj_joint_calculation_) {
    for (int i = 0; i < link_num_-1; ++i) {
      try{
        geometry_msgs::TransformStamped link_transform;
        link_transform = tfBuffer_.lookupTransform(robot_name_+"/link"+std::to_string(i+2), robot_name_+"/link"+std::to_string(i+1), ros::Time(0));
        link_transforms_[i] = link_transform;
      } catch (tf2::TransformException& ex) {
        // TODO
        ROS_WARN("%s",ex.what());
        return;
      }
    }

    for (int i = 0; i < link_num_-1; ++i) {
      R_ci_cache_[i] = TorsionEstimator::quat_to_rot_mat(neuron_imu_data_[i].orientation);
    }
  }

  for (int i = 0; i < link_num_-1; ++i) {
    if (!is_use_orientation_) {
      y_measure[6*i] = neuron_imu_data_[i].linear_acceleration.x;
      y_measure[6*i+1] = neuron_imu_data_[i].linear_acceleration.y;
      y_measure[6*i+2] = neuron_imu_data_[i].linear_acceleration.z;

    } else if (is_use_adj_joint_calculation_) { 

      tf2::Quaternion q_shape, q_org_observed, q_next_observed, q_torsion;
      tf2::convert(link_transforms_[i].transform.rotation, q_shape);
      tf2::convert(neuron_imu_data_[i].orientation, q_org_observed);
      tf2::convert(neuron_imu_data_[i+1].orientation, q_next_observed);

      q_torsion = q_org_observed.inverse() * q_next_observed * q_shape.inverse();

      double roll, pitch, yaw;
      tf2::getEulerYPR(q_torsion, yaw, pitch, roll);

      // Low Pass BiQuad filter
      // filter parameters
      double omega = 2.0 * 3.14159265 *  torsion_cutoff_freq_ / kf_step_rate_;
      double alpha = sin(omega) / (2.0 * torsion_q_);
      double a0 =  1.0 + alpha;
      double a1 = -2.0 * cos(omega);
      double a2 =  1.0 - alpha;
      double b0 = (1.0 - cos(omega)) / 2.0;
      double b1 =  1.0 - cos(omega);
      double b2 = (1.0 - cos(omega)) / 2.0;

      double torsion_in = -roll;
      double torsion = b0/a0 * torsion_in + b1/a0 * torsion_in_prev1_[i]  + b2/a0 * torsion_in_prev2_[i]
        - a1/a0 * torsion_out_prev1_[i] - a2/a0 * torsion_out_prev2_[i];

      torsion_in_prev2_[i] = torsion_in_prev1_[i];
      torsion_in_prev1_[i] = torsion_in;
      torsion_out_prev2_[i] = torsion_out_prev1_[i];
      torsion_out_prev1_[i] = torsion;

      y_measure[6*i] = torsion; // TODO other 2 place would be ignored
      y_measure[6*i+1] = 0;
      y_measure[6*i+2] = 0;

    } else {
      double roll, pitch, yaw;
      TorsionEstimator::geometry_quat_to_rpy(roll, pitch, yaw, neuron_imu_data_[i].orientation);
      y_measure[6*i  ] = roll;
      y_measure[6*i+1] = pitch;
      y_measure[6*i+2] = yaw;
    }

    y_measure[6*i+3] = neuron_imu_data_[i].angular_velocity.x;
    y_measure[6*i+4] = neuron_imu_data_[i].angular_velocity.y;
    y_measure[6*i+5] = neuron_imu_data_[i].angular_velocity.z;

    if (is_use_ext_joint_) {
      y_measure[6*(link_num_-1) + i] = torsions_[i];
    }
  }

  Eigen::VectorXd torsion_vel = Eigen::VectorXd::Zero(link_num_-1);
  if (!is_use_ekf_ && !is_use_ekf_ && is_use_adj_joint_calculation_) {
    /*
    Eigen::MatrixXd R_omega = Eigen::MatrixXd::Zero(3*(link_num_-1), 3*(link_num_-1));
    for (int i = 0; i < link_num_-1; ++i) {
      for (int j = 0; j < link_num_-1-i; ++j) {
        auto& R_ci = R_ci_cache_[i];
        R_omega.block(3*i+3*j, 3*i, 3, 3) = R_ci;
      }
    }
    Eigen::VectorXd omega = Eigen::VectorXd(3*(link_num_-1));
    for (int i = 0; i < link_num_-1; ++i) {
      omega(3*i) = neuron_imu_data_[i].angular_velocity.x - base_gyro_[0];
      omega(3*i+1) = neuron_imu_data_[i].angular_velocity.y - base_gyro_[1];
      omega(3*i+2) = neuron_imu_data_[i].angular_velocity.z - base_gyro_[2];
    }
    omega = R_omega.inverse() * omega;
    for (int i = 0; i < link_num_-1; ++i) {
      torsion_vel(i) = omega(3*i);
    }
    */

    // Low Pass BiQuad filter
    // filter parameters
    double omega = 2.0 * 3.14159265 *  torsion_vel_cutoff_freq_ / kf_step_rate_;
    double alpha = sin(omega) / (2.0 * torsion_vel_q_);

    double a0 =  1.0 + alpha;
    double a1 = -2.0 * cos(omega);
    double a2 =  1.0 - alpha;
    double b0 = (1.0 - cos(omega)) / 2.0;
    double b1 =  1.0 - cos(omega);
    double b2 = (1.0 - cos(omega)) / 2.0;

    for (int i = 0; i < link_num_-1; ++i) {
      Eigen::VectorXd torsion_vel_in = Eigen::VectorXd::Zero(link_num_-1);
      torsion_vel_in(i) = (y_measure[6*i]-prev_torsions_[i])*kf_step_rate_;

      torsion_vel(i) = b0/a0 * torsion_vel_in(i) + b1/a0 * torsion_vel_in_prev1_[i]  + b2/a0 * torsion_vel_in_prev2_[i]
                        - a1/a0 * torsion_vel_out_prev1_[i] - a2/a0 * torsion_vel_out_prev2_[i];

      torsion_vel_in_prev2_[i] = torsion_vel_in_prev1_[i];
      torsion_vel_in_prev1_[i] = torsion_vel_in(i);

      torsion_vel_out_prev2_[i] = torsion_vel_out_prev1_[i];
      torsion_vel_out_prev1_[i] = torsion_vel(i);

      prev_torsions_[i] = y_measure[6*i];
    }
  }

  // joint angles
  for (int i = 0; i < link_num_-1; ++i) {
    u_control[6+3*i] = joints_[i];
    u_control[6+3*i+1] = joints_d_[i];
    u_control[6+3*i+2] = 0;
  }

  // base attitude
  u_control[0] = base_angles_[0];
  u_control[1] = base_angles_[1];
  u_control[2] = base_angles_[2];
  u_control[3] = base_gyro_[0];
  u_control[4] = base_gyro_[1];
  u_control[5] = base_gyro_[2];

  if (is_use_ukf_) {
    ukf_->updateState(u_control, y_measure);
  } else if (is_use_ekf_) {
    ekf_->updateState(u_control, y_measure);
  }

  // publish estimate result
  sensor_msgs::JointState torsion_estimate;
  // TODO add timestamp
  for(int i=0; i<link_num_-1; i++) {
    torsion_estimate.name.push_back("torsion"+std::to_string(i+1));
    if (is_use_ukf_) {
      torsion_estimate.position.push_back(ukf_->getEstimate()(2*i) );
      torsion_estimate.velocity.push_back(ukf_->getEstimate()(2*i+1));
    } else if (is_use_ekf_) {
      torsion_estimate.position.push_back(ekf_->getEstimate()(2*i) );
      torsion_estimate.velocity.push_back(ekf_->getEstimate()(2*i+1));
    } else {
      // TODO
      torsion_estimate.position.push_back( y_measure[6*i] );
      torsion_estimate.velocity.push_back( torsion_vel(i) );
    }
    torsion_estimate.effort.push_back(0);
  }
  estimate_state_pub_.publish(torsion_estimate);
}


void TorsionEstimator::neuronIMUCallback(const sensor_msgs::ImuConstPtr& msg, const int link_id) {
  neuron_imu_data_[link_id].angular_velocity = msg->angular_velocity;
  neuron_imu_data_[link_id].linear_acceleration = msg->linear_acceleration;
  neuron_imu_data_[link_id].orientation = msg->orientation;

  setBaseIMU(link_id);
}

void TorsionEstimator::simMocapCallback(const nav_msgs::OdometryConstPtr& msg, const int link_id) {
  mocapFilter(link_id, msg->pose.pose, msg->header);

  setBaseIMU(link_id);
}

void TorsionEstimator::mocapCallback(const geometry_msgs::PoseStampedConstPtr& msg, const int link_id) {
  mocapFilter(link_id, msg->pose, msg->header);

  setBaseIMU(link_id);
}

void TorsionEstimator::mocapFilter(const int link_id, const geometry_msgs::Pose& pose, const std_msgs::Header& header) {
  double r, p, y, r_p, p_p, y_p;
  TorsionEstimator::geometry_quat_to_rpy(r,p,y,pose.orientation);
  TorsionEstimator::geometry_quat_to_rpy(r_p,p_p,y_p,neuron_imu_data_[link_id].orientation);

  // TODO
  neuron_imu_data_[link_id].angular_velocity.x = (r-r_p)/(header.stamp.nsec - neuron_imu_data_[link_id].header.stamp.nsec)*1e9;
  neuron_imu_data_[link_id].angular_velocity.y = (p-p_p)/(header.stamp.nsec - neuron_imu_data_[link_id].header.stamp.nsec)*1e9;
  neuron_imu_data_[link_id].angular_velocity.z = (y-y_p)/(header.stamp.nsec - neuron_imu_data_[link_id].header.stamp.nsec)*1e9;

  neuron_imu_data_[link_id].header = header;
  neuron_imu_data_[link_id].orientation = pose.orientation;
}

void TorsionEstimator::setBaseIMU(const int link_id) {
  if (!is_use_base_imu_ && link_id == base_link_id_-1) {
    double roll, pitch, yaw;
    TorsionEstimator::geometry_quat_to_rpy(roll, pitch, yaw, neuron_imu_data_[link_id].orientation);
    base_angles_[0] = roll; base_angles_[1] = pitch; base_angles_[2] = yaw;
    base_gyro_[0] = neuron_imu_data_[link_id].angular_velocity.x; base_gyro_[1] = neuron_imu_data_[link_id].angular_velocity.y; base_gyro_[2] = neuron_imu_data_[link_id].angular_velocity.z; 
  }
}

void TorsionEstimator::jointsCallback(const sensor_msgs::JointStateConstPtr& msg) {
  copy(msg->position.begin(), msg->position.end(), joints_.begin());
  copy(msg->velocity.begin(), msg->velocity.end(), joints_d_.begin());
}

void TorsionEstimator::torsionCallback(const sensor_msgs::JointStateConstPtr& msg) {
  copy(msg->position.begin(), msg->position.end(), torsions_.begin());
}

void TorsionEstimator::baseImuCallback(const spinal::ImuConstPtr& msg) {
  for (int i = 0; i < 3; ++i) {
    base_angles_[i] = msg->angles[i];
    base_gyro_[i] = msg->gyro_data[i];
  }
}

void TorsionEstimator::torsionKsysCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  int i = 0;
  int j = 0;
  for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
    torsion_K_sys_matrix_(j, i) = *it;
    i++;
    if (i==model_->q_size) {
      i = 0;
      j++;
    }
    if (j==model_->q_size) {
      break;
    }
  }
}

arma::vec TorsionEstimator::processFunction(const arma::vec& q, const arma::vec& u) {
  // set data for q, qdot
  for (int i = 0; i < link_num_-1; ++i) {
    // TODO sign ok?????
    Q_cache_(torsion_dof_update_order_[i]) = q[2*i];
    QDot_cache_(torsion_dof_update_order_[i]) = q[2*i+1];

    Q_cache_(joint_dof_update_order_[i]) = u[6+3*i];
    QDot_cache_(joint_dof_update_order_[i]) = u[6+3*i+1];
  }

  if (is_use_simple_system_model_) {
    if (is_use_K_sys_) {
      QDDot_cache_ = torsion_K_sys_matrix_ * Q_cache_;
    } else {
      calcTorsionalMoment(Q_cache_, QDot_cache_, QDDot_cache_);
    }
  } else {
    calcTorsionalMoment(Q_cache_, QDot_cache_, Tau_cache_);
    RigidBodyDynamics::ForwardDynamics(*model_, Q_cache_, QDot_cache_, Tau_cache_, QDDot_cache_); // TODO f_ext for more accurate base att motion?
  }

  // forward euler diff
  arma::vec result = arma::ones(state_size_);
  for (int i = 0; i < link_num_-1; ++i) {
    result[2*i] = q[2*i] + q[2*i+1]/kf_step_rate_;
    result[2*i+1] = q[2*i+1] + QDDot_cache_(torsion_dof_update_order_[i])/kf_step_rate_;
  }
  return result;
}

arma::mat TorsionEstimator::processJacobian(const arma::vec& q, const arma::vec& u) {
  arma::mat result;

  if (is_use_simple_system_model_ && is_use_K_sys_) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state_size_, state_size_);
    for (int i = 0; i < link_num_-1; ++i) {
      F.row(2*i) = torsion_K_sys_matrix_.block(torsion_dof_update_order_[i], 3, 1, state_size_);
    }

    Eigen::MatrixXd result_eigen = (F/kf_step_rate_).exp();

    result = TorsionEstimator::cast_arma(F);
  }

  return result;
}

arma::vec TorsionEstimator::outputFunction(const arma::vec& q) {
  arma::vec result = arma::ones(measure_size_);

  if (is_use_simple_system_model_ && is_use_adj_joint_calculation_) {
    for (int i = 0; i < link_num_-1; ++i) {
      result(6*i  ) = q[2*i];
      result(6*i+1) = 0;
      result(6*i+2) = 0;
    }

    Eigen::Vector3d omega(base_gyro_[0], base_gyro_[1], base_gyro_[2]);
    for (int i = 0; i < link_num_-1; ++i) {
      auto& R_ci = R_ci_cache_[i];
      Eigen::Vector3d omega_next( q[2*i+1], 0, 0); // TODO
      omega = omega + R_ci*omega_next;

      result(6*i+3) = omega(0);
      result(6*i+4) = omega(1);
      result(6*i+5) = omega(2);
    }

  } else {
    // set data for q, qdot
    for (int i = 0; i < link_num_-1; ++i) {
      Q_cache_(torsion_dof_update_order_[i]) = q[2*i];
      QDot_cache_(torsion_dof_update_order_[i]) = q[2*i+1];
    }

    // calcTorsionalMoment(Q_cache_, QDot_cache_, Tau_cache_);
    // RigidBodyDynamics::ForwardDynamics(*model_, Q_cache_, QDot_cache_, Tau_cache_, QDDot_cache_); // TODO f_ext for more accurate base att motion?
    QDDot_cache_.setZero();

    RigidBodyDynamics::UpdateKinematics(*model_, Q_cache_, QDot_cache_, QDDot_cache_);

    for (int i = 0; i < link_num_-1; ++i) {
      std::string imu_name =  (neuron_imu_frame_name_prefix_+std::to_string(i+1)+neuron_imu_frame_name_suffix_);
      int body_id = model_->GetBodyId( imu_name.c_str() );
      RigidBodyDynamics::Math::SpatialVector imu_acc
        = RigidBodyDynamics::CalcPointAcceleration6D(*model_, Q_cache_, QDot_cache_, QDDot_cache_, body_id, RigidBodyDynamics::Math::Vector3d::Zero(), false);

      result(6*i  ) = imu_acc(3);
      result(6*i+1) = imu_acc(4);
      result(6*i+2) = imu_acc(5);
      result(6*i+3) = imu_acc(0);
      result(6*i+4) = imu_acc(1);
      result(6*i+5) = imu_acc(2);


      if (is_use_orientation_) {
        // TODO revise and debug the values!
        auto E = RigidBodyDynamics::CalcBodyWorldOrientation(*model_, Q_cache_, body_id, false);
        double roll, pitch, yaw;
        tf::Matrix3x3 E_tf(E(0,0), E(0,1), E(0,2), E(1,0), E(1,1), E(1,2), E(2,0), E(2,1), E(2,2));
        E_tf.getRPY(roll, pitch, yaw);
        result(6*i  ) = roll;
        result(6*i+1) = pitch;
        result(6*i+2) = yaw;
      }
    }

  }

  for (int i = 0; i < link_num_-1; ++i) {
    if (is_use_ext_joint_) {
      result(6*(link_num_-1) + i) = q[2*i];
    }
  }

  return result;
}

arma::mat TorsionEstimator::outputJacobian(const arma::vec& q) {
  Eigen::MatrixXd result=Eigen::MatrixXd::Identity(measure_size_, state_size_);

  if (is_use_simple_system_model_ && is_use_adj_joint_calculation_) {
    for (int i = 0; i < link_num_-1; ++i) {
      auto& R_ci = R_ci_cache_[i];
      for (int j = i; j < link_num_-1; ++j) {
        result.block(6*i+3, 2*i+1, 3, 1) = R_ci.col(0);
      }
    }

  }

  return cast_arma(result);
}

void TorsionEstimator::calcTorsionalMoment(const RigidBodyDynamics::Math::VectorNd& Q, const RigidBodyDynamics::Math::VectorNd& QDot, RigidBodyDynamics::Math::VectorNd& Tau) {
  for (int i = 0; i < link_num_-1; ++i) {
    Tau(torsion_dof_update_order_[i]) = -torsion_constant_ * Q(torsion_dof_update_order_[i]); // TODO add viscosity?
  }
}
