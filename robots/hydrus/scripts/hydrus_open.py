#!/usr/bin/env python

import sys
import time
import rospy
import math
from sensor_msgs.msg import JointState

if __name__ == "__main__":

    rospy.init_node("hydrus_open")

    link_num = rospy.get_param("~link_num", 8)
    duration = rospy.get_param("~duration", 30.0)
    step = rospy.get_param("~step", 100)
    target_position = rospy.get_param("~target_position", [0.6]*(link_num-1))

    joint_control_topic_name = rospy.get_param("~joint_control_topic_name", "/hydrus/joints_ctrl")
    pub = rospy.Publisher(joint_control_topic_name, JointState, queue_size=1)

    joint_state_topic_name = rospy.get_param("~joint_state_topic_name", "/hydrus/joint_states")
    joint_state = rospy.wait_for_message(joint_state_topic_name, JointState)

    joint = JointState()
    joint.position = []
    for i in range(0, link_num - 1):
        joint.position.append(joint_state.position[i])

    for i in range(0,step+1):
        for j in range(0, link_num - 1):
            joint.position[j] = (i*target_position[j] + (step-i)*joint.position[j]) / step
        pub.publish(joint)
        print joint.position
        rospy.sleep(rospy.Duration.from_sec(duration/step))

