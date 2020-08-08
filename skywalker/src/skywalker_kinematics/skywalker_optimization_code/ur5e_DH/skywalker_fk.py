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

def fk(q):
	global mat
	mat=np.matrix


	# ****** Coefficients ******


	global d, a, alph

	d = mat([0.1625, 0, 0, 0.1333, 0.0997, 0.0996]) #ur5e
	a =mat([0 ,-0.425 ,-0.3922 ,0 ,0 ,0]) #ur5e
	alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2,0 ])  #ur5e




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
	def AH( n,th ):

	  T_a = mat(np.identity(4), copy=False)
	  T_a[0,3] = a[0,n-1]
	  T_d = mat(np.identity(4), copy=False)
	  T_d[2,3] = d[0,n-1]


	  Rzt = mat([[cos(th), -sin(th), 0 ,0],
			 [sin(th),  cos(th), 0, 0],
			 [0,               0,              1, 0],
			 [0,               0,              0, 1]],copy=False)
	      
	  Rxa = mat([[1, 0,                 0,                  0],
				 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
				 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
				 [0, 0,                 0,                  1]],copy=False)

	  A_i = T_d * Rzt * T_a * Rxa
	  
	  #print A_i
		    

	  return A_i

	def HTrans(q):  
	  A_1=AH( 1,q[0]  )
	  A_2=AH( 2,q[1]  )
	  A_3=AH( 3,q[2]  )
	  A_4=AH( 4,q[3]  )
	  A_5=AH( 5,q[4]  )
	  A_6=AH( 6,q[5]  )
	  cc = np.array(([-1 ,0 ,0,0],[0 ,-1 ,0,0],[0 ,0 ,1,0],[0 ,0 ,0,1]))
	  #print(cc)
	      
	  T_06=cc *A_1*A_2*A_3*A_4*A_5*A_6
	  #T_06=A_1*A_2*A_3*A_4*A_5*A_6

	  return T_06


	fk = HTrans(q)
	fk_ros = np2ros(fk)
	x_e = fk_ros.position.x
	y_e = fk_ros.position.y
	z_e = fk_ros.position.z
	x_quat = fk_ros.orientation.x
	y_quat = fk_ros.orientation.y
	z_quat = fk_ros.orientation.z
	w_quat = fk_ros.orientation.w
	
	#print x_e
	#print y_e
	#print z_e
	

	return x_e,y_e, z_e,x_quat,y_quat,z_quat,w_quat
	




