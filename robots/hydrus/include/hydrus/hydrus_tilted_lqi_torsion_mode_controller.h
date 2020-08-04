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

#include <hydrus/hydrus_tilted_lqi_controller.h>
#include <dynamic_reconfigure/server.h>
#include <hydrus/LQI_torsionConfig.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <aerial_robot_msgs/SetFloat64Array.h>
#include <vector>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

#define LQI_TORSION_GAIN_FLAG 0
#define LQI_TORSION_MU_P_GAIN 1
#define LQI_TORSION_MU_D_GAIN 2

namespace aerial_robot_control
{
  class HydrusTiltedLQITorsionModeController: public HydrusLQIController
  {
  public:
    HydrusTiltedLQITorsionModeController() {}
    ~HydrusTiltedLQITorsionModeController();

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);

  protected:
    void controlCore() override;
    bool optimalGain() override;
    void publishGain() override;
    void rosParamInit() override;

    ros::Publisher desired_baselink_rot_pub_;

    double trans_constraint_weight_;
    double att_control_weight_;

    double torsional_spring_constant_;
    int torsion_num_; 
    int mode_num_;
    double init_wait_time_;

    std::vector<double> q_mu_;
    std::vector<double> q_mu_d_;
    std::vector<std::vector<double>> q_mu_p_gains_;
    std::vector<std::vector<double>> q_mu_d_gains_;

    dynamic_reconfigure::Server<hydrus::LQI_torsionConfig>::CallbackType dynamic_reconf_func_lqi_torsion_;
    dynamic_reconfigure::Server<hydrus::LQI_torsionConfig>* lqi_torsion_server_;
    void cfgLQITorsionCallback(hydrus::LQI_torsionConfig& config, uint32_t level);

    ros::Subscriber link_torsion_sub_;
    void linkTorsionCallback(const sensor_msgs::JointStateConstPtr& msg);
    std::vector<double> torsions_;
    std::vector<double> torsions_d_;

    ros::Subscriber eigen_sub_, mode_sub_, B_sub_, B_rot_sub_;
    std::vector<double> torsion_eigens_;
    bool use_rbdl_b_rot_;
    Eigen::MatrixXd torsion_mode_matrix_;
    Eigen::MatrixXd torsion_B_matrix_;
    Eigen::MatrixXd torsion_B_rot_matrix_;
    void torsionEigensCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
    void torsionModeCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
    void torsionBCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
    void torsionBRotCallback(const std_msgs::Float32MultiArrayConstPtr& msg);

    ros::ServiceServer q_mu_srv_;
    ros::ServiceServer q_mu_d_srv_;
    bool setQMuCallback(aerial_robot_msgs::SetFloat64Array::Request &req, aerial_robot_msgs::SetFloat64Array::Response &res);
    bool setQMuDCallback(aerial_robot_msgs::SetFloat64Array::Request &req, aerial_robot_msgs::SetFloat64Array::Response &res);
  };
};
