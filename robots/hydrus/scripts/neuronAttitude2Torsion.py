#!/usr/bin/env python

import time
import rospy
import tf
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_inverse


class NeuronAttitude2Torsion(object):
    """NeuronAttitude2Torsion"""

    def __init__(self):
        self.robot_name_ = rospy.get_param("~robot_name", "hydrus")
        self.links_ = rospy.get_param("~links", 6)
        self.update_rate_ = rospy.get_param("~rate", 30)
        self.simulation_ = rospy.get_param("~simulation", True)

        self.tfl_ = tf.TransformListener()

        self.neuron_subs_ = []
        for i in range(self.links_):
            if self.simulation_:
                self.neuron_subs_.append(rospy.Subscriber("/"+self.robot_name_+"/neuron"+str(i+1)+"_imu", Imu, self.imu_callback, [i]))
            else:
                self.neuron_subs_.append(rospy.Subscriber("/"+self.robot_name_+"/neuron"+str(i+1)+"_attitude", Vector3Stamped, self.attitude_callback, [i]))

        self.neuron_num_ = self.links_
        self.attitudes_ = [[0.0, 0.0, 0.0]]*self.neuron_num_
        self.spinal_attitude_ = [0.0]*3
        self.spinal_imu_sub_ = rospy.Subscriber("/"+self.robot_name_+"/attitude", Vector3Stamped, self.spinal_callback)

        self.jointstate_pub_ = rospy.Publisher("/"+self.robot_name_+"/link_torsion/joint_states", JointState, queue_size=5)

    def execute():
        pass

    def tf2quat(self, transform):
        return transform[1]

    def getUpdateRate(self):
        return self.update_rate_

    def getRobotName(self):
        return self.robot_name_

    def getNeuronNum(self):
        return self.neuron_num_

    def attitude_callback(self, msg, args):
        idx = args[0]
        self.attitudes_[idx-1] = [msg.vector.x, msg.vector.y, msg.vector.z]

    def spinal_callback(self, msg):
        self.spinal_attitude_ = [msg.vector.x, msg.vector.y, msg.vector.z]

    def imu_callback(self, msg, args):
        idx = args[0]
        euler_ori = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.attitudes_[idx-1] = euler_ori

class AdjacentAttDiffEstimator(NeuronAttitude2Torsion):
    def __init__(self):
        NeuronAttitude2Torsion.__init__(self)
        self.torsion_num_ = self.neuron_num_-1
        self.torsion_prev_ = [0.0]*self.torsion_num_
        self.torsion_vel_prev_ = [0.0]*self.torsion_num_

    def execute(self):
        jointstate = JointState()
        jointstate.header.stamp = rospy.Time.now()
        # TODO make diff with spinal attitude for tf coherency
        att_quats = []
        for att in self.attitudes_:
            att_quats.append(tf.transformations.quaternion_from_euler(att[0], att[1], att[2]))

        link_tfs = []
        try:
            for i in range(self.links_):
                link_tfs.append(self.tfl_.lookupTransform('/'+self.robot_name_+'/link'+str(i+1), 'world', rospy.Time(0)))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # for i in [2]:
        #     q_org_ideal = self.tf2quat(link_tfs[0])
        #     q_next_ideal = self.tf2quat(link_tfs[i+1])
        #     q_shape = quaternion_multiply(q_next_ideal, quaternion_inverse(q_org_ideal))

        #     q_org_observed = att_quats[0]
        #     q_next_observed = att_quats[i+1]

        #     q_torsion = quaternion_multiply(q_next_observed, quaternion_inverse(quaternion_multiply(q_shape, q_org_observed)))
        for i in range(self.torsion_num_):
            q_org_ideal = self.tf2quat(link_tfs[i])
            q_next_ideal = self.tf2quat(link_tfs[i+1])
            q_shape = quaternion_multiply(q_next_ideal, quaternion_inverse(q_org_ideal))

            q_org_observed = att_quats[i]
            q_next_observed = att_quats[i+1]

            q_torsion = quaternion_multiply(q_next_observed, quaternion_inverse(quaternion_multiply(q_shape, q_org_observed)))
            euler_torsion = euler_from_quaternion(q_torsion)
            torsion = -euler_torsion[0]
            torsion_vel = (torsion-self.torsion_prev_[i])*self.update_rate_
            torsion_vel = (torsion_vel+self.torsion_vel_prev_[i])/2 #simple LPF
            jointstate.name.append("torsion"+str(i+1))
            jointstate.position.append(torsion)
            jointstate.velocity.append(torsion_vel)
            jointstate.effort.append(0.0)
            self.torsion_prev_[i] = torsion
            self.torsion_vel_prev_[i] = torsion_vel
        self.jointstate_pub_.publish(jointstate)

class TipDiffEstimator(NeuronAttitude2Torsion):
    def __init__(self):
        NeuronAttitude2Torsion.__init__(self)
        self.torsion_prev_ = 0.0
        self.torsion_vel_prev_ = 0.0

    def execute(self):
        jointstate = JointState()
        jointstate.header.stamp = rospy.Time.now()

        att_quats = []
        for att in self.attitudes_:
            att_quats.append(quaternion_from_euler(att[0], att[1], att[2]))
        link_tfs = []
        try:
            for i in range(self.links_):
                link_tfs.append(self.tfl_.lookupTransform('/'+self.robot_name_+'/link'+str(i+1), 'world', rospy.Time(0)))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        q_spinal_observed = quaternion_from_euler(self.spinal_attitude_[0], self.spinal_attitude_[1], self.spinal_attitude_[2])

        # is it correct? the control performance is far better without this estimation... all 0 is better!?
        q_root_observed = att_quats[0]
        q_root_ideal = self.tf2quat(link_tfs[0])
        q_root_shape = quaternion_multiply(q_root_ideal, quaternion_inverse(q_spinal_observed))
        q_root_torsion = quaternion_multiply(
                            quaternion_multiply(quaternion_inverse(q_root_shape), q_root_observed)
                            , quaternion_inverse(q_spinal_observed))
        euler_root_torsion = euler_from_quaternion(q_root_torsion)

        q_tip_observed = att_quats[-1]
        q_tip_ideal = self.tf2quat(link_tfs[-1])
        q_tip_shape = quaternion_multiply(q_tip_ideal, quaternion_inverse(q_spinal_observed))
        q_tip_torsion = quaternion_multiply(
                            quaternion_multiply(quaternion_inverse(q_tip_shape), q_tip_observed)
                            , quaternion_inverse(q_spinal_observed))
        euler_tip_torsion = euler_from_quaternion(q_tip_torsion)

        torsion = -(euler_root_torsion[0]-euler_tip_torsion[0])
        torsion_vel = (torsion-self.torsion_prev_)*self.update_rate_
        torsion_vel = (torsion_vel+self.torsion_vel_prev_)/2 #simple LPF

        jointstate.name.append("torsion"+str(self.links_/2))
        jointstate.position.append(torsion)
        jointstate.velocity.append(torsion_vel)
        jointstate.effort.append(0.0)
        self.torsion_prev_ = torsion
        self.torsion_vel_prev_ = torsion_vel
        self.jointstate_pub_.publish(jointstate)

if __name__ == "__main__":
    rospy.init_node("neuronAttitude2Torsion")
    mode = rospy.get_param("~mode", 'tip')
    estimator = NeuronAttitude2Torsion()
    if mode == 'adj':
        estimator = AdjacentAttDiffEstimator()
    elif mode == 'tip':
        estimator = TipDiffEstimator()
    r = rospy.Rate(estimator.getUpdateRate())
    while not rospy.is_shutdown():
        estimator.execute()
        r.sleep()

