#!/usr/bin/env python

import rospy
import math
import tf
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

def odom_callback(msg, framename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z), 
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                rospy.Time.now(), framename, "world")
    return

if __name__ == "__main__":

    rospy.init_node("neuron_odom2jointstate")

    mode = rospy.get_param("~mode", "pos")
    link_num = rospy.get_param("~torsion_num", 1)
    update_rate = rospy.get_param("~update_rate", 30)

    robot_name = rospy.get_param("~robot_name", "hydrus")
    joint_state_topic_name = rospy.get_param("~joint_state_topic_name", "/"+robot_name+"/link_torsion/joint_states")
    odometry_root_topic_name = rospy.get_param("~joint_control_topic_name", "/"+robot_name+"/neuron1_groundtruth")
    odometry_tip_topic_name = rospy.get_param("~joint_control_topic_name", "/"+robot_name+"/neuron6_groundtruth")

    joint_state_pub = rospy.Publisher(joint_state_topic_name, JointState, queue_size=10)
    odom_root_sub = rospy.Subscriber(odometry_root_topic_name, Odometry, odom_callback, odometry_root_topic_name)
    odom_tip_sub = rospy.Subscriber(odometry_tip_topic_name, Odometry, odom_callback, odometry_tip_topic_name)

    listener = tf.TransformListener()

    joint_torsion = 0.0
    joint_torsion_vel = 0.0
    prev_time = rospy.get_time()

    r = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        try:
            joint_torsion_prev = joint_torsion
            joint_torsion_vel_prev = joint_torsion_vel
            if mode == "pos":
                (trans_root,rot_root) = listener.lookupTransform(odometry_root_topic_name, '/'+robot_name+'/link3', rospy.Time(0))
                (trans_tip,rot_tip) = listener.lookupTransform(odometry_tip_topic_name, '/'+robot_name+'/link3', rospy.Time(0))

                joint_torsion = -trans_root[2] / math.sqrt(trans_root[0]**2+trans_root[1]**2) - trans_tip[2] / math.sqrt(trans_tip[0]**2+trans_tip[1]**2)

            elif mode == "rot":
                (trans_root3,rot_root3) = listener.lookupTransform(odometry_root_topic_name, '/'+robot_name+'/link3', rospy.Time(0))
                (trans_tip3,rot_tip3) = listener.lookupTransform(odometry_tip_topic_name, '/'+robot_name+'/link3', rospy.Time(0))

                (trans_root4,rot_root4) = listener.lookupTransform(odometry_root_topic_name, '/'+robot_name+'/link4', rospy.Time(0))
                (trans_tip4,rot_tip4) = listener.lookupTransform(odometry_tip_topic_name, '/'+robot_name+'/link4', rospy.Time(0))

                rot_root_euler3 = tf.transformations.euler_from_quaternion(rot_root3)
                rot_tip_euler3 = tf.transformations.euler_from_quaternion(rot_tip3)
                rot_root_euler4 = tf.transformations.euler_from_quaternion(rot_root4)
                rot_tip_euler4 = tf.transformations.euler_from_quaternion(rot_tip4)
                # TODO not accurate?
                joint_torsion =  ((rot_root_euler3[1]+rot_tip_euler3[1])+(rot_root_euler4[1]+rot_tip_euler4[1]))

            current_time = rospy.get_time()
            # delta_t = current_time-prev_time
            # if delta_t < 1e-5:
            #     delta_t = 1.0/update_rate
            delta_t = 1.0/update_rate
            joint_torsion_vel = (joint_torsion-joint_torsion_prev)/delta_t
            prev_time = current_time
            joint_torsion_vel = (joint_torsion_vel+joint_torsion_vel_prev)/2 # simple LPF

            joint_state  = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = ['torsion']
            joint_state.position = [joint_torsion]
            joint_state.velocity = [joint_torsion_vel]
            joint_state.effort = [0]
            joint_state_pub.publish(joint_state)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        r.sleep()

