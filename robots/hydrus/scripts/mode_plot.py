#!/usr/bin/env python

import sys
import time
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from matplotlib.collections import LineCollection
from matplotlib.cm import ScalarMappable
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import pandas as pd
import seaborn as sns
from pylab import rcParams
import signal

def sig_handler(signum, frame):
    plt.close()
    sys.exit(0)

def main():

    rospy.init_node("hydrus_mode_plot")

    link_num = rospy.get_param("~link_num", 6)

    joint_state_topic_name = rospy.get_param("~joint_state_topic_name", "/hydrus/joint_states")
    mode_eigen_topic_name = rospy.get_param("~mode_eigen_topic_name", "/hydrus/torsion_eigens")
    mode_matrix_topic_name = rospy.get_param("~mode_matrix_topic_name", "/hydrus/torsion_mode_matrix")
    mode_matrix_data = rospy.wait_for_message(mode_matrix_topic_name, Float32MultiArray)
    mode_eigen_data = rospy.wait_for_message(mode_eigen_topic_name, Float32MultiArray)
    joint_state_data = rospy.wait_for_message(joint_state_topic_name, JointState)

    torsion_num = len(mode_matrix_data.data)/(link_num-1)
    print(mode_matrix_data)

    mode_matrix = np.array(mode_matrix_data.data).reshape(torsion_num, link_num-1)
    print("mode matrix")
    print(mode_matrix)

    # figure 1
    plt.figure(1)
    for i in range(torsion_num):
        mode_eigen = mode_eigen_data.data[i]
        mode_freq = np.sqrt(-mode_eigen) /2 /np.pi
        plt.plot(mode_matrix[i], label="mode "+str(i+1)+" , "+str(int(mode_freq))+" Hz")
    plt.legend()
    np.set_printoptions(precision=2, floatmode='maxprec')
    plt.title("shape of each mode: joint state \n"+str(np.array(joint_state_data.position)))
    plt.xlabel("link no.")
    plt.ylabel("magnitude of deformation in each mode")

    k_gain_topic_name = rospy.get_param("~k_gain_topic_name", "/hydrus/aerial_robot_base_node/K_gain_shifted")
    k_gain_data = rospy.wait_for_message(k_gain_topic_name, Float32MultiArray)

    k_gain_matrix = np.array(k_gain_data.data).reshape(link_num, torsion_num*2+12)
    print("k gain")
    print(k_gain_matrix)

    thrust_x = np.zeros(link_num)
    thrust_y = np.zeros(link_num)
    joint_x = np.zeros(link_num-1)
    joint_y = np.zeros(link_num-1)
    link_lines = []
    link_length = rospy.get_param("~link_length", 0.6)
    link_direction = 0.0
    for i in range(link_num-1):
        joint_x[i] = thrust_x[i] + link_length/2 * np.cos(link_direction)
        joint_y[i] = thrust_y[i] + link_length/2 * np.sin(link_direction)

        link_direction = link_direction + joint_state_data.position[i]

        thrust_x[i+1] = joint_x[i] + link_length/2 * np.cos(link_direction)
        thrust_y[i+1] = joint_y[i] + link_length/2 * np.sin(link_direction)

        link_lines.append([[thrust_x[i], thrust_y[i]], [joint_x[i],joint_y[i]]])
        link_lines.append([[joint_x[i],joint_y[i]], [thrust_x[i+1], thrust_y[i+1]]])

    print(link_lines)

    # figure 2
    fig_row_num = rospy.get_param("~k_gain_fig_row_num", 2)
    fig_cm_name = 'bwr'
    fig_cm_name = 'seismic'
    fig, axs = plt.subplots(fig_row_num, k_gain_matrix.shape[1]/fig_row_num, figsize=(30.0, 10.0))
    k_gain_titles = ["z", "z_d", "roll", "roll_d", "pitch", "pitch_d", "yaw", "yaw_d", "z_i", "roll_i", "pitch_i", "yaw_i"]
    for i in range(torsion_num):
        k_gain_titles.append("torsion_M"+str(i+1))
        k_gain_titles.append("torsion_M"+str(i+1)+"_d")
    for i in range(k_gain_matrix.shape[1]):
        ax = axs[i%fig_row_num, i/fig_row_num]
        gain = k_gain_matrix[:,i]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.set_title(k_gain_titles[i] + ", mag: " + '{:.3f}'.format(np.linalg.norm(gain)) )
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)
        if i >= 12:
            mode = mode_matrix[(i-12)/2]
            ax.scatter(joint_x, joint_y, c=mode, s=200*np.power(abs(mode)/max(abs(mode)),1), marker='*',cmap='PRGn')

    axpos = axs[0,0].get_position()
    cbar_ax = fig.add_axes([0.87, axpos.y0, 0.02, axpos.height])
    norm = colors.Normalize(vmin=-1,vmax=1)
    mappable = ScalarMappable(cmap=fig_cm_name, norm=norm)
    mappable._A = []
    cbar = fig.colorbar(mappable,cax=cbar_ax)
    # cbar.set_label("Z")
    plt.subplots_adjust(right=0.85)
    plt.subplots_adjust(wspace=0.1)
    plt.title("magnitude of control feedback gains: joint state \n"+str(np.array(joint_state_data.position)))

    # figure 3
    k_lqi_gain_topic_name = rospy.get_param("~k_lqi_gain_topic_name", "/hydrus/aerial_robot_base_node/K_gain")
    k_lqi_gain_data = rospy.wait_for_message(k_lqi_gain_topic_name, Float32MultiArray)
    k_lqi_gain_matrix = np.array(k_lqi_gain_data.data).reshape(link_num, torsion_num*2+12)

    fig, axs = plt.subplots(fig_row_num, k_gain_matrix.shape[1]/fig_row_num, figsize=(30.0, 10.0))
    for i in range(k_lqi_gain_matrix.shape[1]):
        ax = axs[i%fig_row_num, i/fig_row_num]
        gain = k_lqi_gain_matrix[:,i]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.set_title(k_gain_titles[i] + ", mag: " + '{:.3f}'.format(np.linalg.norm(gain)) )
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)
        if i >= 12:
            mode = mode_matrix[(i-12)/2]
            ax.scatter(joint_x, joint_y, c=mode, s=200*np.power(abs(mode)/max(abs(mode)),1), marker='*',cmap='PRGn')

    axpos = axs[0,0].get_position()
    cbar_ax = fig.add_axes([0.87, axpos.y0, 0.02, axpos.height])
    norm = colors.Normalize(vmin=-1,vmax=1)
    mappable = ScalarMappable(cmap=fig_cm_name, norm=norm)
    mappable._A = []
    cbar = fig.colorbar(mappable,cax=cbar_ax)
    # cbar.set_label("Z")
    plt.subplots_adjust(right=0.85)
    plt.subplots_adjust(wspace=0.1)
    plt.title("magnitude of control feedback gains without null space shift: joint state \n"+str(np.array(joint_state_data.position)))

    # figure 4
    kernel_matrix_topic_name = rospy.get_param("~kernel_matrix_topic_name", "/hydrus/aerial_robot_base_node/B_kernel")
    kernel_matrix_data = rospy.wait_for_message(kernel_matrix_topic_name, Float32MultiArray)
    kernel_matrix = np.array(kernel_matrix_data.data).reshape(link_num, len(kernel_matrix_data.data)/link_num)
    print("kernel matrix")
    print(kernel_matrix)

    fig, axs = plt.subplots(1, kernel_matrix.shape[1], figsize=(30.0, 10.0))
    for i in range(kernel_matrix.shape[1]):
        ax = axs[i]
        gain = kernel_matrix[:,i]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)
    plt.title("kernels: joint state \n"+str(np.array(joint_state_data.position)))

    # figure 5
    fig, axs = plt.subplots(4, max(3, torsion_num, kernel_matrix.shape[1]), figsize=(30.0, 10.0))
    rows = ['LQI Gain', 'LQI Torsion Gain', 'Shifted Gain', 'Kernel Gain']
    for ax, row in zip(axs[:,0], rows):
        ax.set_title(row)

    ## k lqi
    j = 0
    for i in [2,4,6]:
        ax = axs[0, j]
        j+=1
        gain = k_lqi_gain_matrix[:,i]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.set_title(k_gain_titles[i] + ", mag: " + '{:.3f}'.format(np.linalg.norm(gain)) )
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)
    ## k torsion
    j = 0
    for i in range(12, k_lqi_gain_matrix.shape[1], 2):
        ax = axs[1, j]
        j+=1
        gain = k_lqi_gain_matrix[:,i]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.set_title(k_gain_titles[i] + ", mag: " + '{:.3f}'.format(np.linalg.norm(gain)) )
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)
        if i >= 12:
            mode = mode_matrix[(i-12)/2]
            ax.scatter(joint_x, joint_y, c=mode, s=200*np.power(abs(mode)/max(abs(mode)),1), marker='*',cmap='PRGn')
    ## k shifted
    j = 0
    for i in [2,4,6]:
        ax = axs[2, j]
        j+=1
        gain = k_gain_matrix[:,i]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.set_title(k_gain_titles[i] + ", mag: " + '{:.3f}'.format(np.linalg.norm(gain)) )
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)
    ## kernel
    for i in range(kernel_matrix.shape[1]):
        ax = axs[3, i]
        gain = kernel_matrix[:,i]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)

    cbar = fig.colorbar(mappable,cax=cbar_ax)
    fig.tight_layout()
    plt.title("summary of gain shift \n"+str(np.array(joint_state_data.position)))

    # figure 6
    torsion_B_mode_matrix_topic_name = rospy.get_param("~torsion_B_mode_matrix_topic_name", "/hydrus/torsion_B_mode_matrix")
    torsion_B_mode_matrix_data = rospy.wait_for_message(torsion_B_mode_matrix_topic_name, Float32MultiArray)
    torsion_B_mode_matrix = np.array(torsion_B_mode_matrix_data.data).reshape(len(torsion_B_mode_matrix_data.data)/link_num, link_num)
    print('torsion B mode')
    print(torsion_B_mode_matrix)

    fig, axs = plt.subplots(1, torsion_B_mode_matrix.shape[0], figsize=(30.0, 10.0))
    for i in range(torsion_B_mode_matrix.shape[0]):
        ax = axs[i]
        gain = torsion_B_mode_matrix[i,:]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.set_title('B_mode[' + str(i) + "], mag: " + '{:.3f}'.format(np.linalg.norm(gain)) )
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)
    plt.title("torsion B mode matrix: \n"+str(np.array(joint_state_data.position)))

    # figure 7
    torsion_B_matrix_topic_name = rospy.get_param("~torsion_B_matrix_topic_name", "/hydrus/torsion_B_matrix")
    torsion_B_matrix_data = rospy.wait_for_message(torsion_B_matrix_topic_name, Float32MultiArray)
    torsion_B_matrix = np.array(torsion_B_matrix_data.data).reshape(len(torsion_B_matrix_data.data)/link_num, link_num)
    print('torsion B')
    print(torsion_B_matrix)

    fig, axs = plt.subplots(1, torsion_B_matrix.shape[0], figsize=(30.0, 10.0))
    for i in range(torsion_B_matrix.shape[0]):
        ax = axs[i]
        gain = torsion_B_matrix[i,:]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.set_title('B[' + str(i) + "], mag: " + '{:.3f}'.format(np.linalg.norm(gain)) )
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)
    plt.title("torsion B matrix: \n"+str(np.array(joint_state_data.position)))

    # figure 8
    fig, axs = plt.subplots(1, torsion_B_matrix.shape[1], figsize=(30.0, 10.0))
    for i in range(torsion_B_matrix.shape[1]):
        ax = axs[i]
        gain = np.zeros(link_num)
        gain[i] = 1
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.set_title('B[' + str(i) + "], mag: " + '{:.3f}'.format(np.linalg.norm(gain)) )
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), marker='*', cmap='PRGn')

        mode = torsion_B_matrix[:,i]
        ax.scatter(joint_x, joint_y, c=mode, s=200*np.power(abs(mode)/max(abs(mode)),1),cmap=fig_cm_name)
    plt.title("torsion B matrix: \n"+str(np.array(joint_state_data.position)))

    # figure 9
    gain_df = pd.DataFrame(k_gain_matrix, columns=k_gain_titles)
    corr = gain_df.corr()

    plt.figure(9, figsize=(20,20))
    sns.set(color_codes=True, font_scale=1.2)
    ax = sns.heatmap(
        corr, 
        vmin=-1, vmax=1, center=0,
        cmap=sns.diverging_palette(20, 220, n=200),
        square=True,
        annot=True
        )
    ax.set_xticklabels(
        ax.get_xticklabels(),
        rotation=45,
        horizontalalignment='right'
        )
    plt.title("corr of control feedback gains: joint state \n"+str(np.array(joint_state_data.position)))

    plt.show()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, sig_handler)
    sys.exit(main())
