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




def forward_kinem_franka(q):

	global mat
	mat=np.matrix
	# ****** Coefficients ******
	global d1, a2, a3, d4, d5, d6
	d1 =  0.1625
	a2 = -0.425
	a3 = -0.3922
	d4 =  0.1333
	d5 =  0.0997
	d6 =  0.0996

	global d, a, alph

	d = mat([0.1625, 0, 0, 0.1333, 0.0997, 0.0996]) #ur5e
	a =mat([0 ,-0.425 ,-0.3922 ,0 ,0 ,0]) #ur5e
	alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2,0 ])  #ur5e

	
		

	q1 = q[0]
	q2 = q[1]
	q3 = q[2]
	q4 = q[3]
	q5 = q[4]
	q6 = q[5]
	

	def AH( n,q ):

	  T_a = mat(np.identity(4), copy=False)
	  T_a[0,3] = a[0,n-1]
	  T_d = mat(np.identity(4), copy=False)
	  T_d[2,3] = d[0,n-1]


	  Rzt = mat([[cos(q), -sin(q), 0 ,0],
			 [sin(q),  cos(q), 0, 0],
			 [0,               0,              1, 0],
			 [0,               0,              0, 1]],copy=False)
	      
	  Rxa = mat([[1, 0,                 0,                  0],
				 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
				 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
				 [0, 0,                 0,                  1]],copy=False)

	  A_i = T_d * Rzt * T_a * Rxa
	  
	  #print A_i
	    

  	  return A_i
	A_1=AH( 1,q1  )
	A_2=AH( 2,q2  )
	A_3=AH( 3,q3  )
	A_4=AH( 4,q4  )
	A_5=AH( 5,q5  )
	A_6=AH( 6,q6  )
	cc = np.array(([-1 ,0 ,0,0],[0 ,-1 ,0,0],[0 ,0 ,1,0],[0 ,0 ,0,1]))
	HTrans = A_1.dot(A_2).dot(A_3).dot(A_4).dot(A_5).dot(A_6)
	T_fin =cc.dot(HTrans)
	
	print(np.shape(T_fin))
	print(T_fin)
	
	x_tool0 = T_fin[0, 3]-0.26235
	y_tool0 = T_fin[1, 3]+0.1
	z_tool0 = T_fin[2, 3]+0.842
	
	

	R_fin = T_fin[0:3, 0:3]

	return x_tool0, y_tool0, z_tool0, R_fin



	





