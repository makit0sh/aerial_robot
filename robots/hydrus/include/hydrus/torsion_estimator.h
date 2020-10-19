// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <ros/ros.h>
#include <vector>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <hydrus/util/rbdl_util.h>

#include <kalman_filters/unscented_kalman_filter.h>
#include <kalman_filters/extended_kalman_filter.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <spinal/Imu.h>
#include <unsupported/Eigen/MatrixFunctions>


class TorsionEstimator {
public:
  TorsionEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~TorsionEstimator();

private:
  //private functions
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool debug_;
  bool is_simulation_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  std::string robot_name_;

  RigidBodyDynamics::Model* model_;
  std::vector<unsigned int> torsion_dof_update_order_;
  std::vector<unsigned int> joint_dof_update_order_;
  RigidBodyDynamics::Math::VectorNd Q_cache_;
  RigidBodyDynamics::Math::VectorNd QDot_cache_;
  RigidBodyDynamics::Math::VectorNd QDDot_cache_;
  RigidBodyDynamics::Math::VectorNd Tau_cache_; double torsion_constant_; RigidBodyDynamics::Math::MatrixNd torsion_K_sys_matrix_;
  ros::Subscriber K_sys_sub_;
  std::vector<geometry_msgs::TransformStamped> link_transforms_;
  void torsionKsysCallback(const std_msgs::Float32MultiArrayConstPtr& msg);

  kf::UnscentedKalmanFilter* ukf_;
  kf::ExtendedKalmanFilter* ekf_;
  arma::vec processFunction(const arma::vec& q, const arma::vec& u);
  arma::vec outputFunction(const arma::vec& q);
  arma::mat processJacobian(const arma::vec& q, const arma::vec& u);
  arma::mat outputJacobian(const arma::vec& q);
  int link_num_;
  bool is_use_mocap_;
  bool is_use_ukf_;
  bool is_use_ekf_;
  bool is_use_orientation_;
  bool is_use_base_imu_;
  bool is_use_ext_joint_;
  bool is_use_simple_system_model_;
  bool is_use_adj_joint_calculation_;
  bool is_use_K_sys_;
  int base_link_id_;
  int state_size_;
  int control_size_;
  int measure_size_;
  std::vector<Eigen::Matrix3d> R_ci_cache_;
  std::vector<double> prev_torsions_;
  void calcTorsionalMoment(const RigidBodyDynamics::Math::VectorNd& Q, const RigidBodyDynamics::Math::VectorNd& QDot, RigidBodyDynamics::Math::VectorNd& Tau);

  ros::Timer kf_timer_;
  double kf_step_rate_;
  void kfStepCallback(const ros::TimerEvent&);

  std::string neuron_imu_frame_name_prefix_;
  std::string neuron_imu_frame_name_suffix_;

  std::vector<ros::Subscriber> neuron_imu_subs_;
  void setNeuronIMUData();
  void neuronIMUCallback(const sensor_msgs::ImuConstPtr& msg, const int link_id);
  void mocapFilter(const int link_id, const geometry_msgs::Pose& pose, const std_msgs::Header& header);
  void setBaseIMU(const int link_id);
  double torsion_vel_cutoff_freq_;
  double torsion_vel_q_;
  std::vector<double> torsion_vel_in_prev1_;
  std::vector<double> torsion_vel_in_prev2_;
  std::vector<double> torsion_vel_out_prev1_;
  std::vector<double> torsion_vel_out_prev2_;
  double torsion_cutoff_freq_;
  double torsion_q_;
  std::vector<double> torsion_in_prev1_;
  std::vector<double> torsion_in_prev2_;
  std::vector<double> torsion_out_prev1_;
  std::vector<double> torsion_out_prev2_;
  void mocapCallback(const geometry_msgs::PoseStampedConstPtr& msg, const int link_id);
  void simMocapCallback(const nav_msgs::OdometryConstPtr& msg, const int link_id);
  std::vector<sensor_msgs::Imu> neuron_imu_data_;

  ros::Subscriber jointstate_sub_;
  void jointsCallback(const sensor_msgs::JointStateConstPtr& msg);
  std::vector<double> joints_;
  std::vector<double> joints_d_;
  ros::Subscriber torsionstate_sub_;
  void torsionCallback(const sensor_msgs::JointStateConstPtr& msg);
  std::vector<double> torsions_;

  ros::Subscriber baseImu_sub_;
  void baseImuCallback(const spinal::ImuConstPtr& msg);
  std::vector<double> base_angles_;
  std::vector<double> base_gyro_;

  ros::Time kf_prev_step_time_;

  ros::Publisher estimate_state_pub_;


  void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, tf::Quaternion quat){
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
  }
  void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat){
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
  }
  Eigen::Matrix3d quat_to_rot_mat(double& x, double& y, double& z, double&w) {
    Eigen::Matrix3d result;
    result(0,0)= x*x-y*y-z*z+w*w;result(0,1)= 2.0*(x*y-w*z);  result(0,2)= 2.0*(x*z+w*y);
    result(1,0)= 2.0*(x*y+w*z);  result(1,1)= y*y+w*w-x*x-z*z;result(1,2)= 2.0*(y*z-w*x);
    result(2,0)= 2.0*(x*z-w*y);  result(2,1)= 2.0*(y*z+w*x);  result(2,2)= z*z+w*w-x*x-y*y;
    return result;
  }
  Eigen::Matrix3d quat_to_rot_mat(geometry_msgs::Quaternion& q) {
    return quat_to_rot_mat(q.x, q.y, q.z, q.w);
  }

  Eigen::MatrixXd cast_eigen(arma::mat arma_A) {
    Eigen::MatrixXd eigen_B = Eigen::Map<Eigen::MatrixXd>(arma_A.memptr(),
        arma_A.n_rows,
        arma_A.n_cols);
    return eigen_B;
  }

  arma::mat cast_arma(Eigen::MatrixXd eigen_A) {
    arma::mat arma_B = arma::mat(eigen_A.data(), eigen_A.rows(), eigen_A.cols(),
        false, false);
    return arma_B;
  }

};

