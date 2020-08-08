#!/usr/bin/python2

## UR5/UR10 Inverse Kinematics - Ryan Keating Johns Hopkins University


# ***** lib
import numpy as np
from numpy import linalg


import cmath
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi
from geometry_msgs.msg import Pose
import tf.transformations as tf
import csv
from sympy import *
def fk(q):
	q_1 = q[0]
	q_2 = q[1]
	q_3 = q[2]
	q_4 = q[3]
	q_5 = q[4]
	q_6 = q[5]
	
# 	global mat
# 	mat=np.matrix


# 	# ****** Coefficients ******


# 	global d, a, alph

# 	d = mat([0.1625, 0, 0, 0.1333, 0.0997, 0.0996]) #ur5e
# 	a =mat([0 ,-0.425 ,-0.3922 ,0 ,0 ,0]) #ur5e
# 	alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2,0 ])  #ur5e




# 	def np2ros(np_pose):
# 	    """Transform pose from np.array format to ROS Pose format.

# 	    Args:
# 		np_pose: A pose in np.array format (type: np.array)

# 	    Returns:
# 		An HTM (type: Pose).
# 	    """

# 	    # ROS pose
# 	    ros_pose = Pose()

# 	    # ROS position
# 	    ros_pose.position.x = np_pose[0, 3]
# 	    ros_pose.position.y = np_pose[1, 3]
# 	    ros_pose.position.z = np_pose[2, 3]

# 	    # ROS orientation 
# 	    np_q = tf.quaternion_from_matrix(np_pose)
# 	    ros_pose.orientation.x = np_q[0]
# 	    ros_pose.orientation.y = np_q[1]
# 	    ros_pose.orientation.z = np_q[2]
# 	    ros_pose.orientation.w = np_q[3]

# 	    return ros_pose
# 	def AH( n,th ):

# 	  T_a = mat(np.identity(4), copy=False)
# 	  T_a[0,3] = a[0,n-1]
# 	  T_d = mat(np.identity(4), copy=False)
# 	  T_d[2,3] = d[0,n-1]


# 	  Rzt = mat([[cos(th), -sin(th), 0 ,0],
# 			 [sin(th),  cos(th), 0, 0],
# 			 [0,               0,              1, 0],
# 			 [0,               0,              0, 1]],copy=False)
	      
# 	  Rxa = mat([[1, 0,                 0,                  0],
# 				 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
# 				 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
# 				 [0, 0,                 0,                  1]],copy=False)

# 	  A_i = T_d * Rzt * T_a * Rxa
	  
# 	  #print A_i
		    

# 	  return A_i

# 	def HTrans(q):  
# 	  A_1=AH( 1,q[0]  )
# 	  A_2=AH( 2,q[1]  )
# 	  A_3=AH( 3,q[2]  )
# 	  A_4=AH( 4,q[3]  )
# 	  A_5=AH( 5,q[4]  )
# 	  A_6=AH( 6,q[5]  )
# 	  cc = np.array(([-1 ,0 ,0,0],[0 ,-1 ,0,0],[0 ,0 ,1,0],[0 ,0 ,0,1]))
# 	  #print(cc)
	      
# 	  T_06=cc *A_1*A_2*A_3*A_4*A_5*A_6
# 	  #T_06=A_1*A_2*A_3*A_4*A_5*A_6

# 	  return T_06
	d_1, d_2, d_3, d_4, d_5, d_6 = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996]) #ur5e
	a_1, a_2, a_3, a_4, a_5, a_6 = np.array([0 ,-0.425 ,-0.3922 ,0 ,0 ,0]) #ur5e
	alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6 = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2,0 ])  #ur5e



# T_1 = array([ [cos(q_1), -sin(q_1)*cos(alpha_1),  sin(q_1)*sin(alpha_1), a_1*cos(q_1) ],  [sin(q_1), cos(q_1)*cos(alpha_1),  -cos(q_1)*sin(alpha_1), a_1*sin(q_1)], [0.0, sin(alpha_1),  cos(alpha_1), d_1], [0.0, 0.0, 0.0, 1.0] ]    )

	T_1 = Matrix([ [cos(q_1), -sin(q_1)*cos(alpha_1),  sin(q_1)*sin(alpha_1), a_1*cos(q_1) ],  [sin(q_1), cos(q_1)*cos(alpha_1),  -cos(q_1)*sin(alpha_1), a_1*sin(q_1)], [0.0, sin(alpha_1),  cos(alpha_1), d_1], [0.0, 0.0, 0.0, 1.0] ]    )

	T_2 = Matrix([ [cos(q_2), -sin(q_2)*cos(alpha_2),  sin(q_2)*sin(alpha_2), a_2*cos(q_2) ],  [sin(q_2), cos(q_2)*cos(alpha_2),  -cos(q_2)*sin(alpha_2), a_2*sin(q_2)], [0.0, sin(alpha_2),  cos(alpha_2), d_2], [0.0, 0.0, 0.0, 1.0] ]    )

	T_3 = Matrix([ [cos(q_3), -sin(q_3)*cos(alpha_3),  sin(q_3)*sin(alpha_3), a_3*cos(q_3) ],  [sin(q_3), cos(q_3)*cos(alpha_3),  -cos(q_3)*sin(alpha_3), a_3*sin(q_3)], [0.0, sin(alpha_3),  cos(alpha_3), d_3], [0.0, 0.0, 0.0, 1.0] ]    )

	T_4 = Matrix([ [cos(q_4), -sin(q_4)*cos(alpha_4),  sin(q_4)*sin(alpha_4), a_4*cos(q_4) ],  [sin(q_4), cos(q_4)*cos(alpha_4),  -cos(q_4)*sin(alpha_4), a_4*sin(q_4)], [0.0, sin(alpha_4),  cos(alpha_4), d_4], [0.0, 0.0, 0.0, 1.0] ]    )

	T_5 = Matrix([ [cos(q_5), -sin(q_5)*cos(alpha_5),  sin(q_5)*sin(alpha_5), a_5*cos(q_5) ],  [sin(q_5), cos(q_5)*cos(alpha_5),  -cos(q_5)*sin(alpha_5), a_5*sin(q_5)], [0.0, sin(alpha_5),  cos(alpha_5), d_5], [0.0, 0.0, 0.0, 1.0] ]    )

	T_6 = Matrix([ [cos(q_6), -sin(q_6)*cos(alpha_6),  sin(q_6)*sin(alpha_6), a_6*cos(q_6) ],  [sin(q_6), cos(q_6)*cos(alpha_6),  -cos(q_6)*sin(alpha_6), a_6*sin(q_6)], [0.0, sin(alpha_6),  cos(alpha_6), d_6], [0.0, 0.0, 0.0, 1.0] ]    )

	T_offset = Matrix([[-1 ,0 ,0,0],[0 ,-1 ,0,0],[0 ,0 ,1,0],[0 ,0 ,0,1] ])

	T_fin = T_offset*T_1*T_2*T_3*T_4*T_5*T_6

	def np2ros(np_pose):
		"""Transform pose from np.array format to ROS Pose format.

		Args:
		np_pose: A pose in np.array format (type: np.array)

		Returns:
		An HTM (type: Pose).
		"""

		# ROS pose
		ros_pose = Pose()

		# ROS position
		ros_pose.position.x = np_pose[0, 3]
		ros_pose.position.y = np_pose[1, 3]
		ros_pose.position.z = np_pose[2, 3]

		# ROS orientation 
		np_q = tf.quaternion_from_matrix(np_pose)
		ros_pose.orientation.x = np_q[0]
		ros_pose.orientation.y = np_q[1]
		ros_pose.orientation.z = np_q[2]
		ros_pose.orientation.w = np_q[3]

		return ros_pose


	#fk = HTrans(q)
	fk = T_fin
	fk_ros = np2ros(fk)
	x_e = fk_ros.position.x + -0.26235
	y_e = fk_ros.position.y + 0.1
	z_e = fk_ros.position.z + 0.842
	x_quat = fk_ros.orientation.x
	y_quat = fk_ros.orientation.y
	z_quat = fk_ros.orientation.z
	w_quat = fk_ros.orientation.w



	xyz_e  = np.array(([x_e],[y_e],[z_e]))
	#print xyz_e
	x_c = q[7]
	y_c = q[8]
	z_c = 0.0
	xyz_c  = np.array(([x_c],[y_c],[z_c]))
	#print xyz_c
	th_ = q[6]
	R_  = np.array(([cos(th_) ,-sin(th_),0],[sin(th_) ,cos(th_),0],[0 ,0 ,1]))
	#print R_

	xyz_main = np.dot(R_,xyz_e) + xyz_c
	print x_quat
	print y_quat
	print z_quat
	print w_quat


	return xyz_main
		




