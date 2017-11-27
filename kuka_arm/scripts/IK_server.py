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


def generate_homegeneous_transform(alpha, a, d, q):
    # Define Modified DH Transformation matrix
    h_t = Matrix([
        [cos(q),            -sin(q),            0,              a],
        [sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),    -sin(alpha)*d],
        [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),     cos(alpha)*d],
        [0,                 0,                  0,              1]
    ])
    return h_t

def rot_x(q):
    R_x = Matrix([
        [1,     0,          0],
        [0,     cos(q),     -sin(q)],
        [0,     sin(q),     cos(q)]
    ])
    return R_x
    
def rot_y(q):              
    R_y = Matrix([
        [cos(q),    0,      sin(q)],
        [0,         1,      0],
        [-sin(q),   0,      cos(q)]
    ])
    return R_y

def rot_z(q):    
    R_z = Matrix([
        [cos(q),    -sin(q),    0],
        [sin(q),    cos(q),     0],
        [0,         0,          1]
    ])
    return R_z



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        joint_trajectory_point = JointTrajectoryPoint()
        for x in xrange(0, len(req.poses)):
            # IK code starts here

            ### Your FK code here

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

            # Create symbols
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

            # Create Modified DH parameters
            dh_table = {
                alpha0: 0,      a0: 0,      d1: 0.75,   q1: q1,
                alpha1: -pi/2., a1: 0.35,   d2: 0,      q2: q2-pi/2,
                alpha2: 0,      a2: 1.25,   d3: 0,      q3: q3,
                alpha3: -pi/2., a3: -0.054, d4: 1.50,   q4: q4,
                alpha4: pi/2.,  a4: 0,      d5: 0,      q5: q5,
                alpha5: -pi/2., a5: 0,      d6: 0,      q6: q6,
                alpha6: 0,      a6: 0,      d7: 0.303,  q7: 0,
            }

            # Create individual transformation matrices
            T0_1 = generate_homegeneous_transform(alpha0, a0, d1, q1).subs(dh_table)
            T1_2 = generate_homegeneous_transform(alpha1, a1, d2, q2).subs(dh_table)
            T2_3 = generate_homegeneous_transform(alpha2, a2, d3, q3).subs(dh_table)
            T3_4 = generate_homegeneous_transform(alpha3, a3, d4, q4).subs(dh_table)
            T4_5 = generate_homegeneous_transform(alpha4, a4, d5, q5).subs(dh_table)
            T5_6 = generate_homegeneous_transform(alpha5, a5, d6, q6).subs(dh_table)
            T6_G = generate_homegeneous_transform(alpha6, a6, d7, q7).subs(dh_table)

            # Extract rotation matrices from the transformation matrices
            T0_E = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G


            # EE Matrices
            # EE rotation matrix
            r, p, y = symbols('r p y')
            R_x = rot_x(r)
            R_y = rot_y(p)
            R_z = rot_z(y)

            R_EE = R_z * R_y * R_x
            R_Error = R_EE.subs({'r': 0, 'p': -pi/2, 'y': pi})
            # R_Error = R_z.subs(y, pi) * R_y.subs(p, -pi/2)

            R_EE = R_EE * R_Error

            # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            # Position Matrix of end-effector to base framea
            EE_0 = Matrix([[px], [py], [pz]])
            # Calculate WC(wrist center) position matrix to base frame, reference from 'Inverse Kinematics' section, d7 = 0.303
            WC_0 = EE_0 - dh_table[d7] * R_EE[:, 2]
            
            # Calculate joint angles using Geometric IK method, reference from 'Inverse Kinematics Example' section
            theta1 = atan2(WC_0[1], WC_0[0])

            # Using trigonometry, specifically the Cosine Laws, you can calculate theta 2 and theta 3.
            side_a = dh_table[d4]
            # Reference from 'Inverse Kinematics Example'
            temp_r = sqrt(WC_0[0]**2 + WC_0[1]**2) - dh_table[a1]
            temp_s = WC_0[2] - dh_table[d1]
            # Inverse Kinematics with Kuka KR210
            side_b = sqrt(temp_r**2 + temp_s**2)
            side_c = dh_table[a2]
            # interior angle of triangle
            angle_a = acos((side_b**2 + side_c**2 - side_a**2) / (2 * side_b * side_c))
            angle_b = acos((side_a**2 + side_c**2 - side_b**2) / (2 * side_a * side_c))
            # angle_c = acos((side_a**2 + side_b**2 - side_c**2) / (2 * side_a * side_b))

            # calculate theta2
            theta2 = pi/2 - angle_a - atan2(temp_s, temp_r)
            theta2 = theta2.evalf()

            # theta3 = pi/2 - angle_b - atan2(abs(dh_table[a3]), dh_table[d4])
            theta3 = pi/2 - angle_b - 0.036
            theta3 = theta3.evalf()

            # Euler Angles from a Rotation Matrix
            R0_3 = T0_1[:3, :3] * T1_2[:3, :3] * T2_3[:3, :3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv("LU") * R_EE
            # R3_6 = R3_6.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            theta4 = atan2(R3_6[2,2], -R3_6[0,2]).evalf()
            theta5 = atan2(sqrt(R3_6[1,0]**2 + R3_6[1,1]**2), R3_6[1,2]).evalf()
            # theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2]).evalf()
            theta6 = atan2(-R3_6[1,1], R3_6[1,0]).evalf()

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
