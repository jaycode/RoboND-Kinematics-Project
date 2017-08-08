#!/usr/bin/env python
# Python 2.7

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

def rot_x(q):
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,         cos(q),  -sin(q)],
                  [ 0,         sin(q),  cos(q)]])
    
    return R_x
    
def rot_y(q):              
    R_y = Matrix([[ cos(q),        0,  sin(q)],
                  [      0,        1,       0],
                  [-sin(q),        0, cos(q)]])
    
    return R_y

def rot_z(q):    
    R_z = Matrix([[ cos(q),  -sin(q),       0],
                  [ sin(q),   cos(q),       0],
                  [      0,        0,       1]])
    
    return R_z

def trans_matrix(alpha, a, d, q):
    T = Matrix([[            cos(q),           -sin(q),           0,             a],
                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
    return T

def rad(deg):
    """ Convert degree to radian.
    """
    return deg * pi/180.

def dist(original_pos, target_pos):
    """ Find distance from given original and target position.
    """
    vector = target_pos - original_pos
    return sqrt((vector.T * vector)[0])

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        s = {alpha0:        0, a0:      0, d1:  0.75, q1: q1,
             alpha1: rad(-90), a1:   0.35, d2:     0, q2: q2-rad(90),
             alpha2:        0, a2:   1.25, d3:     0, q3: q3,
             alpha3: rad(-90), a3: -0.054, d4:  1.50, q4: q4,
             alpha4:  rad(90), a4:      0, d5:     0, q5: q5,
             alpha5: rad(-90), a5:      0, d6:     0, q6: q6,
             alpha6:        0, a6:      0, d7: 0.303, q7: 0
        }

        # Create individual transformation matrices
        T0_1 = trans_matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = trans_matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = trans_matrix(alpha2, a2, d3, q3).subs(s)
        T3_4 = trans_matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = trans_matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = trans_matrix(alpha5, a5, d6, q6).subs(s)
        T6_EE = trans_matrix(alpha6, a6, d7, q7).subs(s)

        T0_EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)

        # From walkthrough. Update all lines below that use T.
        # No difference in result and performance compared with the above.
        # T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

        # Debug
        # print("T0_1 = ", T0_1.evalf(subs=s))
        # print("T0_2 = ", T0_2.evalf(subs=s))
        # print("T0_3 = ", T0_3.evalf(subs=s))
        # print("T0_4 = ", T0_4.evalf(subs=s))
        # print("T0_5 = ", T0_5.evalf(subs=s))
        # print("T0_6 = ", T0_6.evalf(subs=s))
        # print("T0_G = ", T0_G.evalf(subs=s))

        # Extract rotation matrices from the transformation matrices
        ###

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

            r, p, y = symbols('r p y')

            R_x = rot_x(r)
            R_y = rot_y(p)
            R_z = rot_z(y)
            # Rotation matrix of gripper
            # R_EE = R_x * R_y * R_z
            R_EE = R_z * R_y * R_x
            # print R_G
            # Matrix([[r11, r12, r13],
            #         [r21, r22, r23],
            #         [r31, r32, r33]])

            # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rot_err = rot_z(rad(180)) * rot_y(rad(-90))

            # print Rot_err
            # Matrix([[0,  0, 1],
            #         [0, -1, 0],
            #         [1,  0, 0]])

            R_EE = R_EE * Rot_err
            # print R_EE
            # Matrix([[r13, -r12, r11],
            #         [r23, -r22, r21],
            #         [r33, -r32, r31])

            R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
            # Find original wrist position with formula described in
            # https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/a1abb738-84ee-48b1-82d7-ace881b5aec0
            G = Matrix([[px], [py], [pz]])
            WC = G - (0.303) * R_EE[:, 2]

            # Calculate joint angles using Geometric IK method

            # Relevant lesson:
            # https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/8d553d46-d5f3-4f71-9783-427d4dbffa3a
            theta1 = atan2(WC[1], WC[0])

            a = 1.501 # Found by using "measure" tool in RViz.
            b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + \
                pow((WC[2] - 0.75), 2))
            c = 1.25 # Length of joint 1 to 2.

            alpha = acos((b*b + c*c - a*a) / (2*b*c))
            beta = acos((a*a + c*c - b*b) / (2*a*c))
            gamma = acos((a*a + b*b - c*c) / (2*a*b))

            theta2 = pi/2 - alpha - atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)

            # Look at Z position of -0.054 in link 4 and use it to calculate delta
            delta = 0.036 
            theta3 = pi/2 - (beta + delta)

            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})

            R3_6 = R0_3.inv("LU") * R_EE

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5  = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

            rospy.loginfo("theta1: {}".format(theta1))
            rospy.loginfo("theta2: {}".format(theta2))
            rospy.loginfo("theta3: {}".format(theta3))
            rospy.loginfo("theta4: {}".format(theta4))
            rospy.loginfo("theta5: {}".format(theta5))
            rospy.loginfo("theta6: {}".format(theta6))

            ###

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
    print "Ready to receive an IK request!"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
