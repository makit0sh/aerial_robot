#!/usr/bin/env python

import sys
import time
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

if __name__ == "__main__":

    rospy.init_node("hydrus_mode_plot")

    link_num = rospy.get_param("~link_num", 6)

    joint_state_topic_name = rospy.get_param("~joint_state_topic_name", "/hydrus/joint_states")
    mode_eigen_topic_name = rospy.get_param("~mode_eigen_topic_name", "/hydrus/torsion_eigens")
    mode_matrix_topic_name = rospy.get_param("~mode_matrix_topic_name", "/hydrus/torsion_B_mode_matrix")
    mode_matrix_data = rospy.wait_for_message(mode_matrix_topic_name, Float32MultiArray)
    mode_eigen_data = rospy.wait_for_message(mode_eigen_topic_name, Float32MultiArray)
    joint_state_data = rospy.wait_for_message(joint_state_topic_name, JointState)

    torsion_num = len(mode_matrix_data.data)/link_num
    print(mode_matrix_data)
    print(torsion_num)

    mode_matrix = np.array(mode_matrix_data.data).reshape(torsion_num, link_num)
    print(mode_matrix)

    for i in range(torsion_num):
        mode_eigen = mode_eigen_data.data[i]
        mode_freq = np.sqrt(-mode_eigen) /2 /np.pi
        plt.plot(mode_matrix[i], label="mode "+str(i+1)+" , "+str(int(mode_freq))+" Hz")
    plt.legend()
    np.set_printoptions(precision=2, floatmode='maxprec')
    plt.title("shape of each mode: joint state \n"+str(np.array(joint_state_data.position)))
    plt.xlabel("link no.")
    plt.ylabel("magnitude of deformation in each mode")
    plt.show()
