#!/usr/bin/env python

import sys
import time
import rospy
import math
import csv
import rospkg
from sensor_msgs.msg import JointState

if __name__ == "__main__":

    rospy.init_node("hydrus_open")
    rospack = rospkg.RosPack()

    link_num = rospy.get_param("~link_num", 8)
    is_interactive = rospy.get_param("~is_interactive", False)
    duration = rospy.get_param("~duration", 30.0)
    target_position_filename = rospy.get_param("~target_position_filename", rospack.get_path('hydrus')+"/scripts/joints_ctrl_list.csv")

    joint_control_topic_name = rospy.get_param("~joint_control_topic_name", "/hydrus/joints_ctrl")
    pub = rospy.Publisher(joint_control_topic_name, JointState, queue_size=1)

    joints = []
    with open(target_position_filename) as f:
        reader = csv.reader(f)
        for line in reader:
            joint = JointState()
            for word in line:
                joint.position.append(float(word))
            joints.append(joint)

    rospy.sleep(rospy.Duration.from_sec(duration))
    for joint in joints:
        print(joint.position)
        pub.publish(joint)
        if is_interactive:
            tmp = raw_input('Press Enter to proceed')
            continue
        else:
            rospy.sleep(rospy.Duration.from_sec(duration))

