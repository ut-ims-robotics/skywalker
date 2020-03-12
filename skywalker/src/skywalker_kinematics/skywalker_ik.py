#!/usr/bin/python2

## UR5/UR10 Inverse Kinematics - Ryan Keating Johns Hopkins University

import autograd.numpy as np   # Thinly-wrapped version of Numpy
from autograd import grad
from autograd import jacobian
import autograd
from scipy.linalg import block_diag
import matplotlib.pyplot as plt
from scipy.optimize import Bounds
from scipy.optimize import LinearConstraint
from scipy.optimize import NonlinearConstraint
from scipy.optimize import minimize
import time
# ***** lib
#import numpy as np
#from numpy import linalg


#import cmath
#import math
#from math import cos as cos
#from math import sin as sin
#from math import atan2 as atan2
#from math import acos as acos
#from math import asin as asin
#from math import sqrt as sqrt
#from math import pi as pi
from geometry_msgs.msg import Pose
import tf.transformations as tf
import csv




def ik(x_base,y_base,z_base,th_,theta_min,theta_max,theta_init,x_desired,y_desired,z_desired,desired_rotation_mat):

	def cost_theta(q):

		#global mat
		#mat=np.matrix
		# ****** Coefficients ******
		#global d1, a2, a3, d4, d5, d6
		#d1 =  0.1625
		#a2 = -0.425
		#a3 = -0.3922
		#d4 =  0.1333
		#d5 =  0.0997
		#d6 =  0.0996

		global d, a, alph

		d = np.matrix([0.1625, 0, 0, 0.1333, 0.0997, 0.0996]) #ur5e
		a =np.matrix([0 ,-0.425 ,-0.3922 ,0 ,0 ,0]) #ur5e
		alph = np.matrix([np.pi/2, 0, 0, np.pi/2, -np.pi/2,0 ])  #ur5e

		
			

		q1 = q[0]
		q2 = q[1]
		q3 = q[2]
		q4 = q[3]
		q5 = q[4]
		q6 = q[5]
		#q7 = q[6]
		#x_base = q[7]
		#y_base = q[8]
		
		
	
		def AH( n,q ):

		  T_a = np.matrix(np.identity(4), copy=False)
		  T_a[0,3] = a[0,n-1]
		  T_d = np.matrix(np.identity(4), copy=False)
		  T_d[2,3] = d[0,n-1]


		  Rzt = np.matrix([[np.cos(q), -np.sin(q), 0 ,0],
				 [np.sin(q),  np.cos(q), 0, 0],
				 [0,               0,              1, 0],
				 [0,               0,              0, 1]],copy=False)
		      
		  Rxa = np.matrix([[1, 0,                 0,                  0],
					 [0, np.cos(alph[0,n-1]), -np.sin(alph[0,n-1]),   0],
					 [0, np.sin(alph[0,n-1]),  np.cos(alph[0,n-1]),   0],
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
		#HTrans = A_1.dot(A_2).dot(A_3).dot(A_4).dot(A_5).dot(A_6)
		temp1 = np.dot(A_5, A_6)
		temp2 = np.dot(A_4, temp1)
		temp3 = np.dot(A_3, temp2)
		temp4 = np.dot(A_2, temp3)
		HTrans= np.dot(A_1, temp4)
		T_fin = np.dot(cc,HTrans)
		###
		x_tool0 = T_fin[0, 3]-0.26235
		y_tool0 = T_fin[1, 3]+0.1
		z_tool0 = T_fin[2, 3]+0.842
		R_fin = T_fin[0:3, 0:3]
		#print (R_fin)
		#trace_R_fin = np.trace(R_fin)

		############ this is what you get for a theta
		tr = R_fin[0,0]+R_fin[1,1]+R_fin[2,2]
		ax1 = 0.5*(R_fin[2,1]-R_fin[1,2])
		ax2 = 0.5*(R_fin[0,2]-R_fin[2,0])
		ax3 = 0.5*(R_fin[1,0]-R_fin[0,1])


		############ this is where you want to reac
		tr_actual = desired_rotation_mat[0,0]+desired_rotation_mat[1,1]+desired_rotation_mat[2,2]
		ax1_actual = 0.5*(desired_rotation_mat[2,1]-desired_rotation_mat[1,2])
		ax2_actual = 0.5*(desired_rotation_mat[0,2]-desired_rotation_mat[2,0])
		ax3_actual = 0.5*(desired_rotation_mat[1,0]-desired_rotation_mat[0,1])


		R_theta = np.array(([np.cos(th_) ,-np.sin(th_),0],[np.sin(th_) ,np.cos(th_),0],[0 ,0 ,1]))
		#print R_theta
		x_mobile_main, y_mobile_main, z_mobile_main = np.dot(R_theta, np.hstack((x_tool0,y_tool0,z_tool0 )))+np.hstack(( x_base, y_base, z_base ))

		#f = np.linalg.norm(q-theta_init)		

		cost = 0.001*sum((q-theta_init)**2)+(x_mobile_main-x_desired)**2+(y_mobile_main-y_desired)**2+(z_mobile_main-z_desired)**2+(tr-tr_actual)**2+(ax1-ax1_actual)**2+(ax2-ax2_actual)**2+(ax3-ax3_actual)**2

		
		return cost
	
	cost_theta_grad = grad(cost_theta)

	bounds = Bounds(theta_min, theta_max)

	# options={'maxiter':maxiter}

	theta_guess = theta_init 

	res = minimize(cost_theta, theta_guess, method='SLSQP', jac=cost_theta_grad, bounds=bounds)	
	sol = res.x

	return sol	
