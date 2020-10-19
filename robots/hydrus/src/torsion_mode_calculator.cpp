#include<hydrus/torsion_mode_calculator.h>

TorsionModeCalculator::TorsionModeCalculator(ros::NodeHandle nh, ros::NodeHandle nhp)
:nh_(nh), nhp_(nhp)
{
  // ros param init
  // urdf model of hydrus
  std::string control_model_urdf;
  nhp_.getParam("robot_description_control", control_model_urdf);
  nhp_.param<bool>("floating", is_floating_, true);
  nhp_.param<bool>("root_on_fc", is_root_on_fc_, false);
  nhp_.param<int>("rotor_num", rotor_num_, 6);
  nhp_.param<int>("torsion_num", torsion_num_, rotor_num_-1);
  nhp_.param<double>("torsion_constant", torsion_constant_, 1.0);
  nhp_.param<double>("eigen_eps", eigen_eps_, 1e-5);
  nhp_.param<int>("mode_num", mode_num_, rotor_num_-4); // TODO consider the used up dofs!!
  nhp_.param<double>("m_f_rate", m_f_rate_, -0.0172);
  nhp_.param<int>("link1_rotor_direction", link1_rotor_direction_, -1);

  torsions_.resize(torsion_num_);
  torsions_d_.resize(torsion_num_);
  joints_.resize(torsion_num_);
  joints_d_.resize(torsion_num_);

  model_ = new Model();
  // construct model from urdf
  if (!Addons::URDFReadFromString (control_model_urdf.c_str(), model_, is_floating_)) {
    ROS_ERROR_STREAM("Error loading model " << control_model_urdf);
    abort();
  }
  ROS_DEBUG_STREAM("Degree of freedom overview:" << Utils::GetModelDOFOverview(*model_));
  if (is_root_on_fc_ == false) {
    torsion_dof_update_order_ = rbdl_util::get_torsion_dof_update_order(model_, torsion_num_);
    joint_dof_update_order_ = rbdl_util::get_torsion_dof_update_order(model_, torsion_num_, "link", 2);
  } else {
    torsion_dof_update_order_ = std::vector<unsigned int>{9, 7, 10, 12, 14};  //TODO remove, only for debug
    joint_dof_update_order_ = std::vector<unsigned int>{6, 8, 11, 13, 15};  //TODO remove, only for debug
  }

  // ros subscribers
  torsion_joint_sub_ = nh.subscribe<sensor_msgs::JointState>("link_torsion/joint_states", 1, &TorsionModeCalculator::torsionJointCallback, this);
  joint_sub_ = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, &TorsionModeCalculator::jointsCallback, this);

  // ros publishers
  eigen_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("torsion_eigens", 100);
  B_mode_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("torsion_B_mode_matrix", 100);
  B_trans_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("torsion_B_trans_matrix", 100);
  B_rot_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("torsion_B_rot_matrix", 100);
  mode_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("torsion_mode_matrix", 100);
  K_sys_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("torsion_K_sys_matrix", 100);

  // dynamic reconfigure
  reconf_server_ = new dynamic_reconfigure::Server<hydrus::torsion_modeConfig>(nhp_);
  reconf_func_ = boost::bind(&TorsionModeCalculator::cfgCallback, this, _1, _2);
  reconf_server_->setCallback(reconf_func_);
}

TorsionModeCalculator::~TorsionModeCalculator() {
  delete model_;
}

void TorsionModeCalculator::cfgCallback(hydrus::torsion_modeConfig &config, uint32_t level) {
  if(config.torsion_mode_flag)
  {
    printf("Torsion Mode Param:");
    switch(level)
    {
      case TORSION_CONSTANT:
        torsion_constant_ = config.torsion_constant;
        printf("change parameter of torsion constant: %f\n", torsion_constant_);
        break;
      default :
        printf("\n");
        break;
    }
  }
}

void TorsionModeCalculator::torsionJointCallback(const sensor_msgs::JointStateConstPtr& msg) {
  if (torsion_num_<=0) return;
  copy(msg->position.begin(), msg->position.end(), torsions_.begin());
  copy(msg->velocity.begin(), msg->velocity.end(), torsions_d_.begin()); 
}

void TorsionModeCalculator::jointsCallback(const sensor_msgs::JointStateConstPtr& msg) {
  if (torsion_num_<=0) return;
  copy(msg->position.begin(), msg->position.end(), joints_.begin());
  copy(msg->velocity.begin(), msg->velocity.end(), joints_d_.begin());
}

void TorsionModeCalculator::calculate() {

  int q_joint_bias = is_floating_ ? 6 : 0; // TODO hardcoding!

  VectorNd Q = VectorNd::Zero (model_->q_size);
  VectorNd QDot = VectorNd::Zero (model_->qdot_size);
  VectorNd QDDot = VectorNd::Zero (model_->qdot_size);

  // map torsion to model Q and QDot
  for (int i = 0; i < torsion_num_; ++i) {
    Q(joint_dof_update_order_[i]) = joints_[i];
    QDot(joint_dof_update_order_[i]) = joints_d_[i];
    Q(torsion_dof_update_order_[i]) = torsions_[i];
    QDot(torsion_dof_update_order_[i]) = torsions_d_[i];
  }

  MatrixNd H = MatrixNd::Zero(model_->qdot_size, model_->qdot_size);
  CompositeRigidBodyAlgorithm(*model_, Q, H);
  MatrixNd H_inv = H.inverse();

  MatrixNd B = MatrixNd::Zero(q_joint_bias/2+torsion_num_, rotor_num_);
  for (int i = 0; i < rotor_num_; ++i) {
    // TODO this value is not related with urdf!
    int sigma = ((i%2==0) ? 1 : -1) * link1_rotor_direction_;
    MatrixNd point_jac = MatrixNd::Zero(6, model_->qdot_size);
    CalcPointJacobian6D(*model_, Q, model_->GetBodyId(("thrust"+std::to_string(i+1)).c_str()), Vector3d::Zero(), point_jac);
    MatrixNd thrust_jac = sigma * H_inv * point_jac.transpose().col(5); // thrust is z axis is inverted with rotor direction

    MatrixNd thrust_jac_counter_torque = m_f_rate_ * H_inv * point_jac.transpose().col(2); // thrust counter torque
    thrust_jac = thrust_jac + thrust_jac_counter_torque;

    VectorNd thrust_jac_att_torsion = VectorNd::Zero(q_joint_bias/2+torsion_num_);

    // att part
    if (is_floating_) {
      thrust_jac_att_torsion(0) = thrust_jac(3);
      thrust_jac_att_torsion(1) = thrust_jac(4);
      thrust_jac_att_torsion(2) = thrust_jac(5);
    }
    // torsion part
    for (int j = 0; j < torsion_num_; ++j) {
      thrust_jac_att_torsion(q_joint_bias/2+j) = thrust_jac(torsion_dof_update_order_.at(j));
    }
    B.col(i) = thrust_jac_att_torsion;
  }
  ROS_DEBUG_STREAM("B: " << std::endl << B);

  MatrixNd A = MatrixNd::Zero(q_joint_bias/2+torsion_num_, q_joint_bias/2+torsion_num_);
  MatrixNd K = MatrixNd::Zero(model_->qdot_size, model_->qdot_size);

  for (int i = 0; i < torsion_num_; ++i) {
    if (i == 0) {
      K(torsion_dof_update_order_.at(i), torsion_dof_update_order_.at(i))  =-1;
      K(torsion_dof_update_order_.at(i), torsion_dof_update_order_.at(i+1))= 1;
      continue;
    }
    if (i == torsion_num_ -1) {
      K(torsion_dof_update_order_.at(i), torsion_dof_update_order_.at(i-1))= 1;
      K(torsion_dof_update_order_.at(i), torsion_dof_update_order_.at(i))  =-1;
      continue;
    }
    K(torsion_dof_update_order_.at(i), torsion_dof_update_order_.at(i-1))= 1;
    K(torsion_dof_update_order_.at(i), torsion_dof_update_order_.at(i))  =-2;
    K(torsion_dof_update_order_.at(i), torsion_dof_update_order_.at(i+1))= 1;
  }
  K = torsion_constant_ * K;
  K = H_inv * K;
  for (int i = 0; i < torsion_num_; ++i) {
    for (int j = 0; j < torsion_num_; ++j) {
      A(q_joint_bias/2+i, q_joint_bias/2+j) = K(torsion_dof_update_order_.at(i), torsion_dof_update_order_.at(j));
    }
  }
  if (is_floating_) {
    A.block(0,0,q_joint_bias/2, q_joint_bias/2) = K.block(0,0,q_joint_bias/2, q_joint_bias/2);
    for (int i = 0; i < torsion_num_; ++i) {
      A(0, q_joint_bias/2+i) = K(3, torsion_dof_update_order_.at(i));
      A(1, q_joint_bias/2+i) = K(4, torsion_dof_update_order_.at(i));
      A(2, q_joint_bias/2+i) = K(5, torsion_dof_update_order_.at(i));
      A(q_joint_bias/2+i, 0) = K(torsion_dof_update_order_.at(i), 3);
      A(q_joint_bias/2+i, 1) = K(torsion_dof_update_order_.at(i), 4);
      A(q_joint_bias/2+i, 2) = K(torsion_dof_update_order_.at(i), 5);
    }
  }
  ROS_DEBUG_STREAM("A: " << std::endl << A);

  // vibration mode regression
  MatrixNd A_vib = A.block(q_joint_bias/2, q_joint_bias/2, torsion_num_, torsion_num_);
  Eigen::EigenSolver<MatrixNd> es(A_vib);
  ROS_DEBUG_STREAM("A_vib eigen values: "<<std::endl << es.eigenvalues().real().transpose());
  ROS_DEBUG_STREAM("A_vib eigen vectors: "<<std::endl << es.eigenvectors().real());

  MatrixNd P = es.eigenvectors().real();
  MatrixNd P_extend = MatrixNd::Identity(q_joint_bias/2+torsion_num_, q_joint_bias/2+torsion_num_);
  MatrixNd P_inv_extend = MatrixNd::Identity(q_joint_bias/2+torsion_num_, q_joint_bias/2+torsion_num_);
  P_extend.block(q_joint_bias/2, q_joint_bias/2, torsion_num_, torsion_num_) = P;
  P_inv_extend.block(q_joint_bias/2, q_joint_bias/2, torsion_num_, torsion_num_) = P.inverse();
  MatrixNd A_full_diag = P_inv_extend * A * P_extend;
  MatrixNd B_full_diag = P_inv_extend * B;
  ROS_DEBUG_STREAM("A full diag: "<<std::endl << A_full_diag);
  ROS_DEBUG_STREAM("B full diag: "<<std::endl << B_full_diag);

  // select largest modes
  MatrixNd A_selected = MatrixNd::Zero(q_joint_bias/2+mode_num_, q_joint_bias/2+mode_num_);
  MatrixNd B_selected = MatrixNd::Zero(q_joint_bias/2+mode_num_, rotor_num_);
  std::vector<std::pair<double, int>> sorted_mode_eigen_idx_pair;
  for (int i = 0; i < es.eigenvalues().size(); ++i) {
    double eig_v = es.eigenvalues()(i).real();
    if (eig_v < 0 && eig_v < -eigen_eps_) {
      sorted_mode_eigen_idx_pair.push_back(std::make_pair(eig_v, i));
    }
  }
  std::sort(sorted_mode_eigen_idx_pair.begin(), sorted_mode_eigen_idx_pair.end(), std::greater<std::pair<double, int>>());

  for (int i = 0; i < mode_num_; ++i) {
    for (int j = 0; j < mode_num_; ++j) {
      double val = A_full_diag(q_joint_bias/2+sorted_mode_eigen_idx_pair[i].second, q_joint_bias/2+sorted_mode_eigen_idx_pair[j].second);
      if (abs(val)>eigen_eps_) A_selected(q_joint_bias/2+i,q_joint_bias/2+j) = val;
    }
    if (is_floating_) {
      A_selected.block(0,q_joint_bias/2+i,q_joint_bias/2,1) = A_full_diag.block(0, q_joint_bias/2+sorted_mode_eigen_idx_pair[i].second, q_joint_bias/2, 1);
      A_selected.block(q_joint_bias/2+i,0,1,q_joint_bias/2) = A_full_diag.block(q_joint_bias/2+sorted_mode_eigen_idx_pair[i].second, 0, 1, q_joint_bias/2);
    }
    B_selected.row(q_joint_bias/2+i) = B_full_diag.row(q_joint_bias/2+sorted_mode_eigen_idx_pair[i].second);
  }
  if (is_floating_) {
    A_selected.block(0,0,q_joint_bias/2,q_joint_bias/2) = A_full_diag.block(0,0,q_joint_bias/2,q_joint_bias/2);
    B_selected.block(0,0,q_joint_bias/2,rotor_num_) = B_full_diag.block(0,0,q_joint_bias/2,rotor_num_);
  }

  ROS_DEBUG_STREAM("A mode selected: "<<std::endl << A_selected);
  ROS_DEBUG_STREAM("B mode selected: " <<std::endl<< B_selected);

  // publish results
  std_msgs::Float32MultiArray eigen_msg, B_msg, mode_msg, B_rot_msg, K_sys_msg;
  eigen_msg.data.clear(); B_msg.data.clear(); mode_msg.data.clear(); K_sys_msg.data.clear();
  for (int i = 0; i < mode_num_; ++i) {
    eigen_msg.data.push_back(sorted_mode_eigen_idx_pair[i].first);
    for (int j = 0; j < torsion_num_; ++j) {
      mode_msg.data.push_back(P(j, sorted_mode_eigen_idx_pair[i].second));
    }
  }
  for (int i = q_joint_bias/2; i < B_selected.rows(); ++i) {
    for (int j = 0; j < B_selected.cols(); ++j) {
      B_msg.data.push_back(B_selected(i,j));
    }
  }
  for (int i = 0; i < K.rows(); ++i) {
    for (int j = 0; j < K.cols(); ++j) {
      K_sys_msg.data.push_back(K(i,j));
    }
  }
  eigen_pub_.publish(eigen_msg);
  B_mode_pub_.publish(B_msg);
  mode_pub_.publish(mode_msg);
  K_sys_pub_.publish(K_sys_msg);

  if (is_floating_) {
    for (int i = 0; i < q_joint_bias/2; ++i) {
      for (int j = 0; j < B_selected.cols(); ++j) {
        B_rot_msg.data.push_back(B_selected(i,j));
      }
    }
    B_rot_pub_.publish(B_rot_msg);

    // TODO enable B_trans!!
  }

}

int main (int argc, char* argv[]) {
  rbdl_check_api_version (RBDL_API_VERSION);

  ros::init(argc, argv, "torsion_mode_calculator");
  ros::NodeHandle nh;
  ros::NodeHandle nhp = ros::NodeHandle("~");

  TorsionModeCalculator mode_calculator(nh, nhp);

  int rate;
  nhp.param<int>("rate", rate, 10);
  ros::Rate r(rate);
  while (ros::ok())
  {
    mode_calculator.calculate();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

