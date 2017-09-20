#!/usr/bin/env python

# import modules
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import JointState
import numpy as np

class FK_server():
    def __init__(self):
        rospy.init_node('FK_server')
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.loginfo("FK server ready")

    def tf_matrix(self, alpha, a, d, q):
        tf_m = np.array([[            np.cos(q),                 -np.sin(q),               0,                  a],
                     [np.sin(q) * np.cos(alpha),  np.cos(q) * np.cos(alpha),  -np.sin(alpha), -np.sin(alpha) * d],
                     [np.sin(q) * np.sin(alpha),  np.cos(q) * np.sin(alpha),   np.cos(alpha),  np.cos(alpha) * d],
                     [                        0,                          0,               0,                  1]])
        return tf_m

    def joint_states_callback(self, msg):
        joint_values = msg.position
        DH_table = {'alpha0':          0, 'a0':      0, 'd1':  0.75, 'q1': joint_values[0],
                    'alpha1': -np.pi / 2, 'a1':   0.35, 'd2':     0, 'q2': joint_values[1] - np.pi / 2.,
                    'alpha2':          0, 'a2':   1.25, 'd3':     0, 'q3': joint_values[2],
                    'alpha3': -np.pi / 2, 'a3': -0.054, 'd4':  1.50, 'q4': joint_values[3],
                    'alpha4':  np.pi / 2, 'a4':      0, 'd5':     0, 'q5': joint_values[4],
                    'alpha5': -np.pi / 2, 'a5':      0, 'd6':     0, 'q6': joint_values[5],
                    'alpha6':          0, 'a6':      0, 'd7': 0.303, 'q7': 0}

        ### Homogeneous Transforms between i and i-1 links
        T0_1 = self.tf_matrix(DH_table['alpha0'], DH_table['a0'], DH_table['d1'], DH_table['q1'])
        T1_2 = self.tf_matrix(DH_table['alpha1'], DH_table['a1'], DH_table['d2'], DH_table['q2'])
        T2_3 = self.tf_matrix(DH_table['alpha2'], DH_table['a2'], DH_table['d3'], DH_table['q3'])
        T3_4 = self.tf_matrix(DH_table['alpha3'], DH_table['a3'], DH_table['d4'], DH_table['q4'])
        T4_5 = self.tf_matrix(DH_table['alpha4'], DH_table['a4'], DH_table['d5'], DH_table['q5'])
        T5_6 = self.tf_matrix(DH_table['alpha5'], DH_table['a5'], DH_table['d6'], DH_table['q6'])
        T6_G = self.tf_matrix(DH_table['alpha6'], DH_table['a6'], DH_table['d7'], DH_table['q7'])
        T0_2 = np.dot(T0_1, T1_2)  # tf base_link to link2
        T0_3 = np.dot(T0_2, T2_3)  # tf base_link to link3
        T0_4 = np.dot(T0_3, T3_4)  # tf base_link to link4
        T0_5 = np.dot(T0_4, T4_5)  # tf base_link to link5
        T0_6 = np.dot(T0_5, T5_6) # tf base_link to link6
        T0_G = np.dot(T0_6, T6_G)  # tf base_link to link7

        R_z = np.array([[np.cos(np.pi), -np.sin(np.pi), 0, 0],
                        [np.sin(np.pi),  np.cos(np.pi), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])  # YAW

        R_y = np.array([[np.cos(-np.pi / 2), 0, np.sin(-np.pi / 2), 0],
                        [0, 1, 0, 0],
                        [-np.sin(-np.pi / 2), 0, np.cos(-np.pi / 2), 0],
                        [0, 0, 0, 1]])  # PITCH
        R_corr = np.dot(R_z, R_y)
        T0_total = np.dot(T0_G, R_corr)

        quat = tf.transformations.quaternion_from_matrix(T0_total)

        tf_base_link_to_ee = TransformStamped()
        tf_base_link_to_ee.header.stamp = rospy.Time.now()
        tf_base_link_to_ee.header.frame_id = "link_6"
        tf_base_link_to_ee.child_frame_id = "base_link"

        tf_base_link_to_ee.transform.translation.x = T0_total[0][3]
        tf_base_link_to_ee.transform.translation.y = T0_total[1][3]
        tf_base_link_to_ee.transform.translation.z = T0_total[2][3]
        tf_base_link_to_ee.transform.rotation.x = quat[0]
        tf_base_link_to_ee.transform.rotation.y = quat[1]
        tf_base_link_to_ee.transform.rotation.z = quat[2]
        tf_base_link_to_ee.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tf_base_link_to_ee)
        r = rospy.Rate(10)







        r.sleep()


if __name__ == "__main__":
    FK_server()
    rospy.spin()