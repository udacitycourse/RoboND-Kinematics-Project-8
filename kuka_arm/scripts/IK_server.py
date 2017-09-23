#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np

q1, q2, q3, q4 = symbols('q1:5')

### Define functions for Rotation Matrices about x, y, and z given specific angle.
def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])

    return R_x


def rot_y(q):
    R_y = Matrix([[cos(q), 0, sin(q)],
                  [0, 1, 0],
                  [-sin(q), 0, cos(q)]])

    return R_y


def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q), 0],
                  [0, 0, 1]])

    return R_z


# Define Modified DH Transformation matrix
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[cos(q), -sin(q), 0, a],
                 [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                 [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                 [0, 0, 0, 1]])
    return TF



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Your FK code here
        # Create symbols
        # alpha: twist angle
        # a: link length
        # d: link offsets
        # q: theta variables
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        # KUKA KR210
        dh_table = {alpha0:       0, a0:      0, d1:  0.75, q1: q1,
                    alpha1: -pi / 2, a1:   0.35, d2:     0, q2: q2 - pi / 2.,
                    alpha2:       0, a2:   1.25, d3:     0, q3: q3,
                    alpha3: -pi / 2, a3: -0.054, d4:  1.50, q4: q4,
                    alpha4:  pi / 2, a4:      0, d5:     0, q5: q5,
                    alpha5: -pi / 2, a5:      0, d6:     0, q6: q6,
                    alpha6:       0, a6:      0, d7: 0.303, q7: 0}

        # Homogeneous Transforms between i and i-1 links
        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(dh_table)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(dh_table)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(dh_table)
        T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(dh_table)
        T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(dh_table)
        T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(dh_table)
        T6_G = TF_Matrix(alpha6, a6, d7, q7).subs(dh_table)

        T0_2 = T0_1 * T1_2  # tf base_link to link2
        T0_3 = T0_2 * T2_3  # tf base_link to link3
        T0_4 = T0_3 * T3_4  # tf base_link to link4
        T0_5 = T0_4 * T4_5  # tf base_link to link5
        T0_6 = T0_5 * T5_6  # tf base_link to link6
        T0_G = T0_6 * T6_G  # tf base_link to link7

        # Extract rotation matrices from the transformation matrices
        R_z = Matrix([[cos(pi), -sin(pi), 0],
                      [sin(pi), cos(pi), 0],
                      [0, 0, 1]])  # YAW

        R_y = Matrix([[cos(-pi / 2), 0, sin(-pi / 2)],
                      [0, 1, 0],
                      [-sin(-pi / 2), 0, cos(-pi / 2)]])  # PITCH

        R_corr = R_z * R_y

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            p = Matrix([[px], [py], [pz]])
            d6 = Matrix([[0], [0], [0.193 + 0.11]])
            R_ee = rot_z(yaw) * rot_y(pitch) * rot_x(roll)
            WC = p - R_ee * R_corr * d6

            # Calculate joint angles using Geometric IK method
            theta1 = atan2(WC[1], WC[0])

            B = np.sqrt(float((np.sqrt(float(WC[0] ** 2 + WC[1] ** 2)) - dh_table[a1]) ** 2 + (WC[2] - dh_table[d1]) ** 2))
            A = dh_table[d4]  # link 3 to link 5
            C = dh_table[a2]
            a = acos((B * B + C * C - A * A) / (2 * B * C))
            b = acos((A * A + C * C - B * B) / (2 * A * C))
            c = acos((A * A + B * B - C * C) / (2 * A * B))

            theta2 = pi / 2 - a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = pi / 2 - (b + 0.036)
            theta2 = theta2.evalf()
            theta3 = theta3.evalf()

            T0_3 = T0_1[:3, :3] * T1_2[:3, :3] * T2_3[:3, :3]
            T0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            T3_6 = T0_3.inv("LU") * R_ee * R_corr
            T3_6 = T3_6.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            theta4 = atan2(T3_6[2, 2], -T3_6[0, 2])
            theta4 = theta4.evalf()
            theta5 = atan2(sqrt(T3_6[1, 0] ** 2 + T3_6[1, 1] ** 2), T3_6[1, 2])
            theta5 = theta5.evalf()
            theta6 = atan2(-T3_6[1, 1], T3_6[1, 0])
            theta6 = theta6.evalf()

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()