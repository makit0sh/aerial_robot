#ifndef AERIAL_ROBOT_BASE_H
#define AERIAL_ROBOT_BASE_H

#include <ros/ros.h>
#include <aerial_robot_base/flight_control.h>
#include <aerial_robot_base/flight_navigation.h>
#include <aerial_robot_base/state_estimation.h>
#include <boost/thread.hpp>
#include <iostream>

using namespace std;

class AerialRobotBase
{
 public:
  AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~ AerialRobotBase();

  void rosParamInit();
  void mainFunc(const ros::TimerEvent & e);
  void tfPubFunc(); //for thread


 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Timer  main_timer_;

  double main_rate_;
  double tf_pub_rate_;
  int motor_num_;

  boost::thread tf_thread_;

  FlightCtrlInput* flight_ctrl_input_;
  PidController* controller_;
  RigidEstimator*  estimator_;
  TeleopNavigator* navigator_;

};


#endif
