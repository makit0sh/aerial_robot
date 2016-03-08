#include <di_control/di_gimbal_control.h>

GimbalControl::GimbalControl(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
{
  gimbalModulesInit();

  alt_control_pub_ = nh_.advertise<aerial_robot_base::FlightNav>("flight_nav", 1);
  desire_tilt_pub_ = nh_.advertise<geometry_msgs::Vector3>("desire_tilt", 1);

  attitude_sub_ = nh_.subscribe<jsk_stm::JskImu>("imu", 1, &GimbalControl::attitudeCallback, this, ros::TransportHints().tcpNoDelay());

  desire_attitude_sub_ = nh_.subscribe<geometry_msgs::Vector3>("desire_attitude", 1, &GimbalControl::desireAttitudeCallback, this, ros::TransportHints().tcpNoDelay());

  attitude_command_sub_ = nh_.subscribe<aerial_robot_base::FourAxisPid>("attitude_command", 1, &GimbalControl::attCommandCallback, this, ros::TransportHints().tcpNoDelay());

  //debug for passive att compare method
  att_diff_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("att_command_current_compare", 1);

  //init servo angle command
  for(int i = 0; i < gimbal_module_num_; i++)
    {
      for(int j = 0; j < 2; j ++)
        {
          std_msgs::Float64 command;
          command.data = gimbal_modules_[i].angle_offset[j];
          gimbal_modules_[i].servos_ctrl_pub[j].publish(command);
        }
    }


  // active gimbal mode param
  nhp_.param("active_gimbal_tilt_duration", active_gimbal_tilt_duration_, 4.0);
  nhp_.param("active_gimbal_tilt_interval", active_gimbal_tilt_interval_, 0.04);

  //dynamic reconfigure server
  gimbal_server_ = new dynamic_reconfigure::Server<di_control::GimbalDynReconfConfig>(nhp_);
  dyn_reconf_func_ = boost::bind(&GimbalControl::GimbalDynReconfCallback, this, _1, _2);
  gimbal_server_->setCallback(dyn_reconf_func_);

  nhp_.param("control_rate", control_rate_, 20.0);
  control_timer_ = nhp_.createTimer(ros::Duration(1.0 / control_rate_), &GimbalControl::controlFunc, this);

}

GimbalControl::~GimbalControl(){};

void GimbalControl::gimbalModulesInit()
{
  nhp_.param("gimbal_module_num", gimbal_module_num_, 4);
  nhp_.param("gimbal_mode", gimbal_mode_, 0);
  nhp_.param("gimbal_thre", gimbal_thre_, 0.0);

  nhp_.param("gimabal_debug", gimbal_debug_, false);

  nhp_.param("body_diameter", body_diameter_, 1.2);

  nhp_.param("att_control_rate", att_control_rate_, 40.0);
  nhp_.param("att_comp_duration", att_comp_duration_, 0.5);
  nhp_.param("passive_level_back_duration", passive_level_back_duration_, 10.0);
  nhp_.param("attitude_outlier_thre", attitude_outlier_thre_, 0.5);

  att_comp_duration_size_ = att_control_rate_ * att_comp_duration_;

  gimbal_modules_.resize(gimbal_module_num_);

  final_attitude_.x = 0;
  final_attitude_.y = 0;
  desire_attitude_.x = 0;
  desire_attitude_.y = 0;

  passive_tilt_mode_ = false;

  roll_diff_ = 0; 
  pitch_diff_ = 0; 
  roll_delay_ = 0; 
  pitch_delay_ = 0; 

  for(int i = 0; i < gimbal_module_num_; i++)
    {
      std::stringstream module_no;
      module_no << i + 1;

      nhp_.param(std::string("gimbal_module") + module_no.str() + std::string("_rotate_angle"), gimbal_modules_[i].rotate_angle, 0.0);

      for(int j = 0; j < 2; j ++)
        {
          std::stringstream servo_no;
          servo_no << 2 * i + j + 1;

          gimbal_modules_[i].servos_ctrl_pub_name[j] = std::string("/j") + servo_no.str()  + std::string("_controller/command");
          gimbal_modules_[i].servos_ctrl_pub[j] = nh_.advertise<std_msgs::Float64>(gimbal_modules_[i].servos_ctrl_pub_name[j], 1); 

          gimbal_modules_[i].servos_state_sub_name[j] = std::string("/j") + servo_no.str()  + std::string("_controller/state");
          gimbal_modules_[i].servos_state_sub[j] = nh_.subscribe<dynamixel_msgs::JointState>(gimbal_modules_[i].servos_state_sub_name[j], 1, boost::bind(&GimbalControl::servoCallback, this, _1, i, j)); //reverse

          gimbal_modules_[i].servos_torque_enable_service_name[j] = std::string("/j") + servo_no.str()  + std::string("_controller/torque_enable");
          gimbal_modules_[i].servos_torque_enable_client[j] = nh_.serviceClient<dynamixel_controllers::TorqueEnable>(gimbal_modules_[i].servos_torque_enable_service_name[j]);

          //torque enable
           dynamixel_controllers::TorqueEnable srv;
           srv.request.torque_enable = true;
           if (gimbal_modules_[i].servos_torque_enable_client[j].call(srv))
             {
               ROS_INFO("no.%d torque_enable", 2 * i + j + 1);
             }
           else
             {
               ROS_ERROR("Failed to call service torque enable");
             }

          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_max"), gimbal_modules_[i].angle_max[j], 1.5 * M_PI ); 
          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_min"), gimbal_modules_[i].angle_min[j], 0.5 * M_PI ); 
          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_sgn"), gimbal_modules_[i].angle_sgn[j], 1);

          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_offset"), gimbal_modules_[i].angle_offset[j], M_PI); 

        }

    }

}

void GimbalControl::attCommandCallback(const aerial_robot_base::FourAxisPidConstPtr& cmd_msg)
{
  boost::lock_guard<boost::mutex> lock(queue_mutex_);
  while (att_command_qu_.size() >= att_comp_duration_size_)
    {
      att_command_qu_.pop_front();
    }

  geometry_msgs::Vector3 new_command;
  new_command.x = cmd_msg->roll.total[0];
  new_command.y = cmd_msg->pitch.total[0];

  att_command_qu_.push_back(new_command);
}

void GimbalControl::desireAttitudeCallback(const geometry_msgs::Vector3ConstPtr& msg)
{
  final_attitude_ = *msg;

}

void GimbalControl::attitudeCallback(const jsk_stm::JskImuConstPtr& msg)
{
  current_attitude_ = msg->angles;

  att_comp_time_ = msg->header.stamp;
}


void GimbalControl::servoCallback(const dynamixel_msgs::JointStateConstPtr& msg, int i, int j)
{
  gimbal_modules_[i].current_angle[j] = gimbal_modules_[i].angle_sgn[j] * (msg->current_pos - gimbal_modules_[i].angle_offset[j]);
}

void GimbalControl::gimbalControl()
{

  Eigen::Quaternion<double> q_att;
  static Eigen::Quaternion<double> prev_q_att = Eigen::Quaternion<double>(1,0,0,0);
  static float tilt_alt_sum = 0;

  if (gimbal_debug_)
    {
      q_att = Eigen::AngleAxisd(current_attitude_.y, Eigen::Vector3d::UnitY()) 
        * Eigen::AngleAxisd(current_attitude_.x, Eigen::Vector3d::UnitX());
    }
  else
    {
      q_att = Eigen::AngleAxisd(desire_attitude_.y, Eigen::Vector3d::UnitY()) 
        * Eigen::AngleAxisd(desire_attitude_.x, Eigen::Vector3d::UnitX());
    }

  //gimbal
  for(int i = 0 ; i < gimbal_module_num_; i++)
    {
      Eigen::Quaternion<double> q =  q_att * Eigen::AngleAxisd(gimbal_modules_[i].rotate_angle, Eigen::Vector3d::UnitZ());
      Eigen::Matrix3d rotation = q.matrix();
      float roll = atan2(rotation(2,1),rotation(2,2));
      float pitch = atan2(-rotation(2,0),  sqrt(rotation(2,1)*rotation(2,1) + rotation(2,2)* rotation(2,2)));

      std_msgs::Float64 command;
      command.data = gimbal_modules_[i].angle_offset[0] - gimbal_modules_[i].angle_sgn[0] * pitch;
      gimbal_modules_[i].servos_ctrl_pub[0].publish(command);

      command.data = gimbal_modules_[i].angle_offset[1] - gimbal_modules_[i].angle_sgn[1] * roll;
      gimbal_modules_[i].servos_ctrl_pub[1].publish(command);
    }

  //alt
  Eigen::Matrix3d r = q_att.matrix();
  Eigen::Matrix3d prev_r = prev_q_att.matrix();
  //float alt_tilt = sin(atan2(sqrt(r(0,2) * r(0,2)  + r(1,2) * r(1,2)), fabs(r(2,2)))) * body_diameter;
  float alt_tilt = sqrt(r(0,2) * r(0,2)  + r(1,2) * r(1,2));
  float prev_alt_tilt = sqrt(prev_r(0,2) * prev_r(0,2)  + prev_r(1,2) * prev_r(1,2));

  aerial_robot_base::FlightNav flight_nav_msg;
  flight_nav_msg.header.stamp = ros::Time::now();
  flight_nav_msg.command_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
  flight_nav_msg.pos_z_navi_mode = aerial_robot_base::FlightNav::VEL_FLIGHT_MODE_COMMAND;
  flight_nav_msg.target_pos_diff_z = body_diameter_ / 2 * (alt_tilt - prev_alt_tilt);
  alt_control_pub_.publish(flight_nav_msg);

  tilt_alt_sum += flight_nav_msg.target_pos_diff_z;


  //ROS_INFO("tilt_alt: %f, sum: %f", flight_nav_msg.target_pos_diff_z, tilt_alt_sum);

  prev_q_att = q_att;
}

void GimbalControl::controlFunc(const ros::TimerEvent & e)
{
  static ros::Time prev_update_time = ros::Time::now();

  static ros::Time passive_tilt_start_time = ros::Time::now();

  if (gimbal_debug_)
    {
      gimbalControl();
      return;
    }


  if(gimbal_mode_ == ACTIVE_GIMBAL_MODE || gimbal_mode_ == ACTIVE_GIMBAL_MODE + PASSIVE_GIMBAL_MODE)
    {
      //duration processing => should be erro estimation processing
      if(ros::Time::now().toSec() - prev_update_time.toSec() < active_gimbal_tilt_duration_) return;

      if(final_attitude_.x - desire_attitude_.x > active_gimbal_tilt_interval_)
        desire_attitude_.x += active_gimbal_tilt_interval_;
      else if (final_attitude_.x - desire_attitude_.x < -active_gimbal_tilt_interval_)
        desire_attitude_.x -= active_gimbal_tilt_interval_;
      else desire_attitude_.x = final_attitude_.x;

      if(final_attitude_.y - desire_attitude_.y > active_gimbal_tilt_interval_)
        desire_attitude_.y += active_gimbal_tilt_interval_;
      else if (final_attitude_.y - desire_attitude_.y < -active_gimbal_tilt_interval_)
        desire_attitude_.y -= active_gimbal_tilt_interval_;
      else desire_attitude_.y = final_attitude_.y;

      ROS_INFO("desire roll: %f, desire pitch: %f", desire_attitude_.x, desire_attitude_.y);
      gimbalControl();

      prev_update_time = ros::Time::now();

      //publish to control board for desire tilt angle
      desire_tilt_pub_.publish(desire_attitude_);

      if(gimbal_mode_ == ACTIVE_GIMBAL_MODE + PASSIVE_GIMBAL_MODE)
        {
          if(desire_attitude_.y == final_attitude_.y && desire_attitude_.x == final_attitude_.x)
            {
              gimbal_mode_ = PASSIVE_GIMBAL_MODE;
              ROS_WARN("back to PASSIVE_GIMBAL_MODE");
            }
        }

    }
  else if(gimbal_mode_ == PASSIVE_GIMBAL_MODE)
    {
      geometry_msgs::Vector3Stamped comp_msg;
      comp_msg.header.stamp = att_comp_time_;

      if(!attCommandCompare()) return;

      comp_msg.vector.x = roll_diff_;
      comp_msg.vector.y = pitch_diff_;
      comp_msg.vector.z = 0;

      if(fabs(roll_diff_) > gimbal_thre_ ||
         fabs(pitch_diff_) > gimbal_thre_)
        {
          final_attitude_ = current_attitude_ ;
          desire_attitude_ = final_attitude_;
          gimbalControl();
          ROS_WARN("big att change in passive mode: roll_diff: %f, roll_delay:%f, pitch_diff: %f, pitch_delay: %f", roll_diff_, roll_delay_, pitch_diff_, pitch_delay_);

          comp_msg.vector.z = 1;
          passive_tilt_mode_ = true;
          passive_tilt_start_time = ros::Time::now();
        }
      att_diff_pub_.publish(comp_msg);

      if( passive_tilt_mode_ && ros::Time::now().toSec() - passive_tilt_start_time.toSec() > passive_level_back_duration_)
        {
          passive_tilt_mode_ = false;
          final_attitude_.x = 0;
          final_attitude_.y = 0;

          gimbal_mode_ = ACTIVE_GIMBAL_MODE + PASSIVE_GIMBAL_MODE;
          ROS_WARN("in order to back to level, turn to acitve tilt mode");
        }
    }
}

bool GimbalControl::attCommandCompare()
{
  boost::lock_guard<boost::mutex> lock(queue_mutex_);

  static geometry_msgs::Vector3 previous_attitude_;

  //outlier attitude data
  if(fabs(previous_attitude_.x - current_attitude_.x) > attitude_outlier_thre_ ||
     fabs(previous_attitude_.y - current_attitude_.y) > attitude_outlier_thre_)
    return false;

  //no enough buffer to compare
  if(att_command_qu_.size() < att_comp_duration_size_) return false;

  int i = 0;
  float min_roll_time = 0, min_pitch_time = 0;
  float min_roll_diff = 1e6, min_pitch_diff = 1e6;
  for(std::deque<geometry_msgs::Vector3>::iterator itr = att_command_qu_.begin(); itr != att_command_qu_.end(); ++itr) 
    {
      //roll
      if(fabs((*itr).x - current_attitude_.x) <  min_roll_diff)
        {
          min_roll_diff = fabs((*itr).x - current_attitude_.x);
          min_roll_time = (att_comp_duration_size_- i ) * (1 / att_control_rate_);
        }
      //pitch
      if(fabs((*itr).y - current_attitude_.y) <  min_pitch_diff)
        {
          min_pitch_diff = fabs((*itr).y - current_attitude_.y);
          min_pitch_time = (att_comp_duration_size_-i ) * (1 / att_control_rate_);
        }

      i++;
    }

  roll_diff_ = min_roll_diff; 
  pitch_diff_ = min_pitch_diff; 
  roll_delay_ = min_roll_time; 
  pitch_delay_ = min_pitch_time; 

  return true;
}

void GimbalControl::GimbalDynReconfCallback(di_control::GimbalDynReconfConfig &config, uint32_t level)
{
  if(config.gimbal_control_flag)
    {
      active_gimbal_tilt_interval_ = config.active_gimbal_tilt_interval;
      active_gimbal_tilt_duration_ = config.active_gimbal_tilt_duration;
      ROS_WARN("new gimbal interval :%f, new gimbal duration :%f", active_gimbal_tilt_interval_, active_gimbal_tilt_duration_);
    }
}