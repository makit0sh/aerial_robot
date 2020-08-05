#!/bin/bash

rosrun aerial_robot_base rosbag_control_data.sh ${1:-hydrus} ${1:-hydrus}/zed/odom ${1:-hydrus}/realsense1/odom/throttle ${1:-hydrus}/realsense1/odom ${1:-hydrus}/realsense2/odom/throttle ${1:-hydrus}/realsense2/odom ${1:-hydrus}/link_torsion/joint_states ${1:-hydrus}/torsion_eigens ${1:-hydrus}/torsion_mode_matrix ${1:-hydrus}/torsion_B_mode_matrix ${1:-hydrus}/torsion_mode_matrix ${@:2}
