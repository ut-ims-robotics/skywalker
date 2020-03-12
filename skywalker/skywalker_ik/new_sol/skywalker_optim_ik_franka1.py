

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


def optim_ik_franka(theta_init, x_e, y_e, z_e, R_e, theta_min, theta_max):

	def cost_theta(q):

		alpha1 = 0
		alpha2 = np.pi/2
		alpha3 = 0.0
		alpha4 = 0.0
		alpha5 = np.pi/2
		alpha6 = -np.pi/2
		alpha7 =  0.0

		d1 = 0.842
		d2 = 0.1625
		d3 = 0.0				
		d4 = 0.0
		d5 = 0.1333
		d6 = 0.0997
		d7 = 0.0996
		
		
		

		a1 = -0.26235	
		a2 = 0.0
		a3 = -0.425
		a4 = -0.3922
		a5 = 0.0
		a6 = 0.0
		a7 = 0.0
		
			

		q1 = q[0]	
		q2 = q[1]
		q3 = q[2]
		q4 = q[3]
		q5 = q[4]
		q6 = q[5]
		q7 = q[6]
		



		T1 = np.array([ [np.cos(q1), -np.sin(q1), 0.0, a1  ], [ np.sin(q1)*np.cos(alpha1), np.cos(q1)*np.cos(alpha1), -np.sin(alpha1), -d1*np.sin(alpha1)  ], [ np.sin(q1)*np.sin(alpha1), np.cos(q1)*np.sin(alpha1), np.cos(alpha1), d1*np.cos(alpha1)   ], [0.0, 0.0, 0.0, 1.0 ]   ])

		T2 = np.array([ [np.cos(q2), -np.sin(q2), 0.0, a2  ], [ np.sin(q2)*np.cos(alpha2), np.cos(q2)*np.cos(alpha2), -np.sin(alpha2), -d2*np.sin(alpha2)  ], [ np.sin(q2)*np.sin(alpha2), np.cos(q2)*np.sin(alpha2), np.cos(alpha2), d2*np.cos(alpha2)   ], [0.0, 0.0, 0.0, 1.0 ]   ])

		T3 = np.array([ [np.cos(q3), -np.sin(q3), 0.0, a3  ], [ np.sin(q3)*np.cos(alpha3), np.cos(q3)*np.cos(alpha3), -np.sin(alpha3), -d3*np.sin(alpha3)  ], [ np.sin(q3)*np.sin(alpha3), np.cos(q3)*np.sin(alpha3), np.cos(alpha3), d3*np.cos(alpha3)   ], [0.0, 0.0, 0.0, 1.0 ]   ])

		T4 = np.array([ [np.cos(q4), -np.sin(q4), 0.0, a4  ], [ np.sin(q4)*np.cos(alpha4), np.cos(q4)*np.cos(alpha4), -np.sin(alpha4), -d4*np.sin(alpha4)  ], [ np.sin(q4)*np.sin(alpha4), np.cos(q4)*np.sin(alpha4), np.cos(alpha4), d4*np.cos(alpha4)   ], [0.0, 0.0, 0.0, 1.0 ]   ])

		T5 = np.array([ [np.cos(q5), -np.sin(q5), 0.0, a5  ], [ np.sin(q5)*np.cos(alpha5), np.cos(q5)*np.cos(alpha5), -np.sin(alpha5), -d5*np.sin(alpha5)  ], [ np.sin(q5)*np.sin(alpha5), np.cos(q5)*np.sin(alpha5), np.cos(alpha5), d5*np.cos(alpha5)   ], [0.0, 0.0, 0.0, 1.0 ]   ])

		T6 = np.array([ [np.cos(q6), -np.sin(q6), 0.0, a6  ], [ np.sin(q6)*np.cos(alpha6), np.cos(q6)*np.cos(alpha6), -np.sin(alpha6), -d6*np.sin(alpha6)  ], [ np.sin(q6)*np.sin(alpha6), np.cos(q6)*np.sin(alpha6), np.cos(alpha6), d6*np.cos(alpha6)   ], [0.0, 0.0, 0.0, 1.0 ]   ])

		T7 = np.array([ [np.cos(q7), -np.sin(q7), 0.0, a7  ], [ np.sin(q7)*np.cos(alpha7), np.cos(q7)*np.cos(alpha7), -np.sin(alpha7), -d7*np.sin(alpha7)  ], [ np.sin(q7)*np.sin(alpha7), np.cos(q7)*np.sin(alpha7), np.cos(alpha7), d7*np.cos(alpha7)   ], [0.0, 0.0, 0.0, 1.0 ]   ])

		#T8 = np.array([ [np.cos(q8), -np.sin(q8), 0.0, a8  ], [ np.sin(q8)*np.cos(alpha8), np.cos(q8)*np.cos(alpha8), -np.sin(alpha8), -d8*np.sin(alpha8)  ], [ np.sin(q8)*np.sin(alpha8), np.cos(q8)*np.sin(alpha8), np.cos(alpha8), d8*np.cos(alpha8)   ], [0.0, 0.0, 0.0, 1.0 ]   ])



		# T_fin = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7).dot(T8)

		#temp1 = np.dot(T7, T8)
		#temp2 = np.dot(T6, temp1)
		#temp3 = np.dot(T5, temp2)
		#temp4 = np.dot(T4, temp3)
		#temp5 = np.dot(T3, temp4)
		#temp6 = np.dot(T2, temp5)
		#T_fin = np.dot(T1, temp6)
		temp1 = np.dot(T6, T7)
		temp2 = np.dot(T5, temp1)
		temp3 = np.dot(T4, temp2)
		temp4 = np.dot(T3, temp3)
		temp5 = np.dot(T2, temp4)
		T_fin = np.dot(T1, temp5)
		
		
		
		#R1 = T1[0:3, 0:3]
		#R2 = T2[0:3, 0:3]
		#R3 = T3[0:3, 0:3]
		#R4 = T4[0:3, 0:3]
		#R5 = T5[0:3, 0:3]
		#R6 = T6[0:3, 0:3]
		

		
		
			
		vec_x = T_fin[0][3]
		vec_y = T_fin[1][3]
		vec_z = T_fin[2][3]
		#vec_x = T_fin[0][3]-0.26235
		#vec_y = T_fin[1][3]+0.1
		#vec_z = T_fin[2][3]+0.842
		

		R= T_fin[0:3, 0:3]

		# trace_R_fin = np.trace(R_fin)

		tr = R[0,0]+R[1,1]+R[2,2]
		ax1 = 0.5*(R[2,1]-R[1,2])
		ax2 = 0.5*(R[0,2]-R[2,0])
		ax3 = 0.5*(R[1,0]-R[0,1])


		tr_actual = R_e[0,0]+R_e[1,1]+R_e[2,2]
		ax1_actual = 0.5*(R_e[2,1]-R_e[1,2])
		ax2_actual = 0.5*(R_e[0,2]-R_e[2,0])
		ax3_actual = 0.5*(R_e[1,0]-R_e[0,1])

		

		#f = np.linalg.norm(q-theta_init)
		f = np.dot((q-theta_init).T, (q-theta_init))

		cost = 1000*(vec_x-x_e)**2+1000*(vec_y-y_e)**2+1000*(vec_z-z_e)**2+(tr-tr_actual)**2+(ax1-ax1_actual)**2+(ax2-ax2_actual)**2+(ax3-ax3_actual)**2+0.1*f

		return cost

	

	cost_theta_grad = grad(cost_theta)

	bounds = Bounds(theta_min, theta_max)

	# options={'maxiter':maxiter}

	theta_guess = theta_init 

	res = minimize(cost_theta, theta_guess, method='SLSQP', jac=cost_theta_grad, bounds=bounds)	
	sol = res.x

	return sol


	





