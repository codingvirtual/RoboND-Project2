#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
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

def calculate_rot_rpw(roll, pitch, yaw):
    roll = Matrix([[1, 0, 0],
                            [0, cos(roll), -sin(roll)],
                            [0, sin(roll), cos(roll)]])

    pitch = Matrix([[cos(pitch), 0, sin(pitch)],
                             [0, 1, 0],
                             [-sin(pitch), 0, cos(pitch)]])

    yaw = Matrix([[cos(yaw), -sin(yaw), 0],
                           [sin(yaw), cos(yaw), 0],
                           [0, 0, 1]])
    return roll, pitch, yaw


def dh_transformation(theta_x, d_dz, theta_z, d_dx):
    return Matrix([[cos(theta_z), -sin(theta_z), 0, d_dz],
                   [sin(theta_z) * cos(theta_x), cos(theta_z) * cos(theta_x), -sin(theta_x), -sin(theta_x) * d_dx],
                   [sin(theta_z) * sin(theta_x), cos(theta_z) * sin(theta_x), cos(theta_x), cos(theta_x) * d_dx],
                   [0, 0, 0, 1]])


def do_rotation(axis, angle=None):
    if axis == "x":
        return Matrix([[1, 0, 0],
                       [0, cos(angle), -sin(angle)],
                       [0, sin(angle), cos(angle)]])
    elif axis == "y":
        return Matrix([[cos(angle), 0, sin(angle)],
                       [0, 1, 0],
                       [-sin(angle), 0, cos(angle)]])
    elif axis == "z":
        return Matrix([[cos(angle), -sin(angle), 0],
                       [sin(angle), cos(angle), 0],
                       [0, 0, 1]])
    else:
        return None


def corr_matrix():
    r_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                  [sin(pi), cos(pi), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    r_y = Matrix([[cos(-pi / 2), 0, sin(-pi / 2), 0],
                  [0, 1, 0, 0],
                  [-sin(-pi / 2), 0, cos(-pi / 2), 0],
                  [0, 0, 0, 1]])
    return simplify(r_z * r_y)


def calc_wrist_center(p_ee, rot_0_6):
    return simplify(p_ee - 0.303 * rot_0_6 * Matrix([[1], [0], [0]]))


def calc_theta1(joint_5):
    return atan2(joint_5[1], joint_5[0])


def calc_theta2(joint5, joint2, theta3_intern):
    link2_3_start = 1.25
    joint3_start = [0.35, 0, 2]
    joint5_start = [1.85, 0, 1.946]

    link3_5_start_x = joint5_start[0] - joint3_start[0]
    link3_5_start_z = joint5_start[2] - joint3_start[2]
    link3_5_start = sqrt(link3_5_start_x ** 2 + link3_5_start_z ** 2)

    dist_1 = link3_5_start * sin(theta3_intern)
    dist_2 = link2_3_start - link3_5_start * cos(theta3_intern)
    alpha = atan2(dist_1, dist_2)
    beta = atan2(joint5[2] - joint2[2], sqrt((joint5[0] - joint2[0]) ** 2 + (joint5[1] - joint2[1]) ** 2))
    theta2 = pi / 2 - alpha - beta
    return theta2

def calc_theta3(joint5, theta1):
    joint3_start = [0.35, 0, 2]
    joint5_start = [1.85, 0, 1.946]
    joint_2 = get_joint2(theta1)
    link2_5_x = joint5[0] - joint_2[0]
    link2_5_y = joint5[1] - joint_2[1]
    link2_5_z = joint5[2] - joint_2[2]
    link2_5 = sqrt(link2_5_x ** 2 + link2_5_y ** 2 + link2_5_z ** 2)
    link2_3_start = 1.25
    link3_5_start = joint5_start[0] - joint3_start[0]
    link3_5_start_z = joint5_start[2] - joint3_start[2]
    link3_5_start = sqrt(link3_5_start ** 2 + link3_5_start_z ** 2)
    distance = (link2_5 ** 2 - link2_3_start ** 2 - link3_5_start ** 2) / -(
            2 * link2_3_start * link3_5_start)
    theta3_intern = atan2(sqrt(1 - distance ** 2), distance)
    theta3 = pi / 2 - (
            atan2(sqrt(1 - distance ** 2), distance) - atan2(link3_5_start_z, link3_5_start))
    return theta3_intern, theta3

def calc_theta6(rotation3_6):
    return atan2(rotation3_6[1, 0], rotation3_6[0, 0])  # rotation about Z-axis


def calc_theta5(rotation3_6):
    return atan2(-rotation3_6[2, 0], sqrt(
        rotation3_6[0, 0] * rotation3_6[0, 0] + rotation3_6[1, 0] * rotation3_6[1, 0]))  # rotation about Y-axis


def calc_theta4(rotation3_6):
    return atan2(rotation3_6[2, 1], rotation3_6[2, 2])  # rotation about X-axis


def get_joint2(theta1):
    joint2_starting_point = [0.35, 0, 0.75]
    joint2 = [joint2_starting_point[0] * cos(theta1), joint2_starting_point[0] * sin(theta1), joint2_starting_point[2]]
    return joint2

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        # link lengths
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        # link offsets
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        # Twist Angles(Alpha)
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        rospy.loginfo("Created DH parameter symbols")

        # Joint angle symbols(Theta)
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        rospy.loginfo("Created Joint Angle symbols")

	    # Create Modified DH parameters
        s = {alpha0: 0, a0: 0, d1: 0.75,
             alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
             alpha2: 0, a2: 1.25, d3: 0,
             alpha3: -pi / 2, a3: -0.054, d4: 1.5,
             alpha4: pi / 2, a4: 0, d5: 0,
             alpha5: -pi / 2, a5: 0, d6: 0,
             alpha6: 0, a6: 0, d7: 0.303, q7: 0}
        rospy.loginfo("Created DH parameters")


    	# Define Modified DH Transformation matrix
        # Create individual transformation matrices
        t0_1 = dh_transformation(alpha0, a0, q1, d1).subs(s)
        t1_2 = dh_transformation(alpha1, a1, q2, d2).subs(s)
        t2_3 = dh_transformation(alpha2, a2, q3, d3).subs(s)
        t3_4 = dh_transformation(alpha3, a3, q4, d4).subs(s)
        t4_5 = dh_transformation(alpha4, a4, q5, d5).subs(s)
        t5_6 = dh_transformation(alpha5, a5, q6, d6).subs(s)
        t6_7 = dh_transformation(alpha6, a6, q7, d7).subs(s)
        t0_7 = simplify(t0_1 * t1_2 * t2_3 * t3_4 * t4_5 * t5_6 * t6_7)
        rospy.loginfo("Created individuals transformation matrices")

	    # Extract rotation matrices from the transformation matrices
        rot_corr = corr_matrix()
        calc_fwd_kin = simplify(t0_7 * rot_corr)
        t0_3 = simplify(t0_1 * t1_2 * t2_3)
        rot_0_3 = t0_3[0:3, 0:3]
        rospy.loginfo("Extracted rot matrices and calculated FK")

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

            # Calculate joint angles using Geometric IK method
            roll, pitch, yaw = calculate_rot_rpw(roll, pitch, yaw)
            rot_0_6 = simplify(roll * pitch * yaw)

            p_end_effector = Matrix([[px], [py], [pz]])
            joint5 = calc_wrist_center(p_end_effector, rot_0_6)

            theta1 = calc_theta1(joint5)
            theta3_intern, theta3 = calc_theta3(joint5, theta1)

            joint2 = get_joint2(theta1)
            theta2 = calc_theta2(joint2, joint5, theta3_intern)


	        # Compensate for rotation discrepancy between DH parameters and Gazebo
            rot_0_3_matrix = rot_0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            rot_0_3_matrix_inverse = rot_0_3_matrix ** -1

            rot_3_6 = rot_0_3_matrix_inverse * rot_0_6
            theta6 = calc_theta6(rot_3_6)
            theta5 = calc_theta5(rot_3_6)
            theta4 = calc_theta4(rot_3_6)

            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            forward_kinematics = calc_fwd_kin.evalf(
                subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
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
