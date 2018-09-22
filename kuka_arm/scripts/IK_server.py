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
import numpy as np
from numpy import array
from sympy import   symbols, cos, sin, pi, sqrt, atan2  

#Create symbol table
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # Link Offset
a0, a1, a2, a3, a4, a5, a6 = symbols ('a0:7') # distance between z(i)_axis and z(i-1)_axis
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #Angle between Z(i-) and Z(i)
DH_table = {alpha0:       0, a0:      0,    d1:  0.75,
                	   alpha1:   -90.0, a1:   0.35,    d2:     0, q2: q2-90.0,
                	   alpha2:       0, a2:   1.25,    d3:     0,
                	   alpha3:   -90.0, a3: -0.054,    d4:   1.5,
                	   alpha4:    90.0, a4:      0,    d5:     0,
                	   alpha5:   -90.0, a5:      0,    d6:     0,
                	   alpha6:       0, a6:      0,    d7: 0.303, q7: 0}
ROT_EE = Matrix([[0,0,0],[0,0,0],[0,0,0]])





        #Modified DH Transformation matrix Function
def TM_Generator(alpha,a,d,q):
	
        tm = Matrix([[             cos(q),             -sin(q),           0,               a],
                     	 [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                     	 [sin(q) * sin(alpha), sin(alpha) * cos(q),  cos(alpha),  cos(alpha) * d],
                    	 [                  0,                   0,           0,               1]])

        return tm
   # Create individual transformation matrices

T0_1 = TM_Generator(alpha0, a0, d1, q1).subs(DH_table)
T1_2 = TM_Generator(alpha1, a1, d2, q2).subs(DH_table)
T2_3 = TM_Generator(alpha2, a2, d3, q3).subs(DH_table)
T3_4 = TM_Generator(alpha3, a3, d4, q4).subs(DH_table)
T4_5 = TM_Generator(alpha4, a4, d5, q5).subs(DH_table)
T5_6 = TM_Generator(alpha5, a5, d6, q6).subs(DH_table)
T6_G = TM_Generator(alpha6, a6, d7, q7).subs(DH_table)

T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

        #Fixing gripper rotation in Y axis by 180 and Z axis by 90

r, p , y = symbols('r p y')
x_rot = Matrix([
            [   1,      0,       0],
            [   0, cos(r), -sin(r)],
            [   0, sin(r),  cos(r)]])  # ROLL

y_rot = Matrix([
            [ cos(p), 0, sin(p)],
            [      0, 1,      0],
            [-sin(p), 0, cos(p)]])  # PITCH

z_rot = Matrix([
            [cos(y), -sin(y), 0],
            [sin(y),  cos(y), 0],
            [     0,       0, 1]])  # YAW


Rot_Fixed = z_rot.subs(y, radians(180)) * y_rot.subs(p,radians(-90))
ROT_Error = z_rot * y_rot * x_rot
ROT_EE = ROT_Error * Rot_Fixed


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print ("No valid poses received")
        return -1
    else:
	global q1, q2, q3, q4, q5, q6, q7 
	global d1, d2, d3, d4, d5, d6, d7 
	global a0, a1, a2, a3, a4, a5, a6 
	global alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 
	global DH_table 
	global ROT_EE
	global T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_G, T0_G

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

            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw}) # Extract nx, ny, and nz values 

            EE_Point = Matrix([[px], [py], [pz]])

            WC = EE_Point - (0.303) * ROT_EE[:,2] # Getting Wrist center coordinates

            side_a = 1.501 #constant
            side_bz = WC[2] - 0.75
	    side_bxy = sqrt(pow(WC[0], 2) + pow(WC[1], 2) ) - 0.35 
	    side_b = sqrt(pow(side_bz, 2) + pow(side_bxy, 2))
            side_c = 1.25  #constant

            angle_a = acos( ( pow(side_b, 2) + pow(side_c, 2) - pow(side_a, 2)) / (2 * side_b * side_c) )
            angle_b = acos( ( pow(side_a, 2) + pow(side_c, 2) - pow(side_b, 2)) / (2 * side_a * side_c) )
            angle_c = acos( ( pow(side_a, 2) + pow(side_b, 2) - pow(side_c, 2)) / (2 * side_a * side_b) )
            

	    theta1 = atan2(WC[1], WC[0])
            theta2 = pi/2 - angle_a - atan2(side_bz, side_bxy) 
            theta3 = pi/2 - (angle_b + 0.036) # 0.036 sag in link4 

            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})

            R3_6 = R0_3.transpose() * ROT_EE

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(pow(R3_6[0,2], 2) + pow(R3_6[2,2], 2)), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

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
