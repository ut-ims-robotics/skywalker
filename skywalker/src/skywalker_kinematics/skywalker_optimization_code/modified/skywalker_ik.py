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
import bernstein_coeff_order10_arbitinterval
from mpl_toolkits.mplot3d import Axes3D
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




#def ik(min_limit,max_limit,theta_init,x_desired,y_desired,z_desired):

#def cost_theta(q):
def obj_fun(c):
	global xe,ye,ze
	global d, a, alph

	d = np.matrix([0.1625, 0, 0, 0.1333, 0.0997, 0.0996]) #ur5e
	a =np.matrix([0 ,-0.425 ,-0.3922 ,0 ,0 ,0]) #ur5e
	alph = np.matrix([np.pi/2, 0, 0, np.pi/2, -np.pi/2,0 ])  #ur5e

	
	c_q1 = c[0:nvar]
	c_q2 = c[nvar:2*nvar]
	c_q3 = c[2*nvar:3*nvar]
	c_q4 = c[3*nvar:4*nvar]
	c_q5 = c[4*nvar:5*nvar]
	c_q6 = c[5*nvar:6*nvar]
	c_x =  c[6*nvar:7*nvar]
	c_y =  c[7*nvar:8*nvar]
	c_phi = c[8*nvar:9*nvar]

	# acc_mat = diff(Pdot, axis = 0)


	# q1_acc = dot(acc_mat, c_q1)
	# q2_acc = dot(acc_mat, c_q2)
	# q3_acc = dot(acc_mat, c_q3)
	# q4_acc = dot(acc_mat, c_q4)
	# q5_acc = dot(acc_mat, c_q5)
	# q6_acc = dot(acc_mat, c_q6)
	# q7_acc = dot(acc_mat, c_q7)

	q_1 = np.dot(P, c_q1)
	q_2 = np.dot(P, c_q2)
	q_3 = np.dot(P, c_q3)
	q_4 = np.dot(P, c_q4)
	q_5 = np.dot(P, c_q5)
	q_6 = np.dot(P, c_q6)
	
	q_1_dot = np.dot(Pdot, c_q1)
	q_2_dot = np.dot(Pdot, c_q2)
	q_3_dot = np.dot(Pdot, c_q3)
	q_4_dot = np.dot(Pdot, c_q4)
	q_5_dot = np.dot(Pdot, c_q5)
	q_6_dot = np.dot(Pdot, c_q6)
	
	q_1_ddot = np.dot(Pddot, c_q1)
	q_2_ddot = np.dot(Pddot, c_q2)
	q_3_ddot = np.dot(Pddot, c_q3)
	q_4_ddot = np.dot(Pddot, c_q4)
	q_5_ddot = np.dot(Pddot, c_q5)
	q_6_ddot = np.dot(Pddot, c_q6)
	## X_base and Y_base and phi
	xo = np.dot(P, c_x)
	yo = np.dot(P, c_y)
	phi = np.dot(P, c_phi)

	# x_base_jerk = dot(jerk_mat, c_x)
	# y_base_jerk = dot(jerk_mat, c_y)

	x_base_vel = np.dot(Pdot, c_x)
	y_base_vel = np.dot(Pdot, c_y)

	x_base_acc = np.dot(Pddot, c_x)
	y_base_acc = np.dot(Pddot, c_y)
	################ non holonomic np.cost	
	eq_nonhol = x_base_vel*np.sin(phi)-y_base_vel*np.cos(phi)

	
	

	# q1 = q[0]
	# q2 = q[1]
	# q3 = q[2]
	# q4 = q[3]
	# q5 = q[4]
	# q6 = q[5]
	# th_ = q[6]
	# x_base = q[7]
	# y_base = q[8]
	# #z_base = q[9]	

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
	A_1=AH( 1,q_1  )
	A_2=AH( 2,q_2  )
	A_3=AH( 3,q_3  )
	A_4=AH( 4,q_4  )
	A_5=AH( 5,q_5  )
	A_6=AH( 6,q_6  )
	cc = np.array(([-1 ,0 ,0,0],[0 ,-1 ,0,0],[0 ,0 ,1,0],[0 ,0 ,0,1]))
	#HTrans = A_1.dot(A_2).dot(A_3).dot(A_4).dot(A_5).dot(A_6)
	T_fin = cc*A_1*A_2*A_3*A_4*A_5*A_6
	# temp1 = np.dot(A_5, A_6)
	# temp2 = np.dot(A_4, temp1)
	# temp3 = np.dot(A_3, temp2)
	# temp4 = np.dot(A_2, temp3)
	# HTrans= np.dot(A_1, temp4)
	# T_fin = np.dot(cc,HTrans)

	#print T_fin
	###
	x_e_l = T_fin[0, 3]
	#print x_e_l
	y_e_l = T_fin[1, 3]
	z_e_l = T_fin[2, 3]

	xc = -0.26235
	yc = 0.1
	zc = 0.842
	
	#x_tool0 = T_fin[0, 3]-0.26235
	#y_tool0 = T_fin[1, 3]+0.1
	#z_tool0 = T_fin[2, 3]+0.842
	xe = (np.cos(phi)*(x_e_l+xc)-np.sin(phi)*(y_e_l+yc)  )
	#print xe
	ye = (np.sin(phi)*(x_e_l+xc)+np.cos(phi)*(y_e_l+yc)  )
	ze = z_e_l+zc

	eq_x = xe+xo-xf_traj
	eq_y = ye+yo-yf_traj
	eq_z = z_e_l+zc-zf_traj

	eq_q1_boundary = q_1[0]-q_1[-1]
	eq_q2_boundary = q_2[0]-q_2[-1]
	eq_q3_boundary = q_3[0]-q_3[-1]
	eq_q4_boundary = q_4[0]-q_4[-1]
	eq_q5_boundary = q_5[0]-q_5[-1]
	eq_q6_boundary = q_6[0]-q_6[-1]



	eq_q1dot_boundary = q_1_dot[0]-q_1_dot[-1]
	eq_q2dot_boundary = q_2_dot[0]-q_2_dot[-1]
	eq_q3dot_boundary = q_3_dot[0]-q_3_dot[-1]
	eq_q4dot_boundary = q_4_dot[0]-q_4_dot[-1]
	eq_q5dot_boundary = q_5_dot[0]-q_5_dot[-1]
	eq_q6dot_boundary = q_6_dot[0]-q_6_dot[-1]
		
		
	eq_q1ddot_boundary = q_1_ddot[0]-q_1_ddot[-1]
	eq_q2ddot_boundary = q_2_ddot[0]-q_2_ddot[-1]
	eq_q3ddot_boundary = q_3_ddot[0]-q_3_ddot[-1]
	eq_q4ddot_boundary = q_4_ddot[0]-q_4_ddot[-1]
	eq_q5ddot_boundary = q_5_ddot[0]-q_5_ddot[-1]
	eq_q6ddot_boundary = q_6_ddot[0]-q_6_ddot[-1]

	eq_xdot_boundary = x_base_vel[0]-x_base_vel[-1]
	eq_ydot_boundary = y_base_vel[0]-y_base_vel[-1]
	eq_xddot_boundary = x_base_acc[0]-x_base_acc[-1]
	eq_yddot_boundary = y_base_acc[0]-y_base_acc[-1]
	eq_x_boundary = xo[0]-xo[-1]
	eq_y_boundary = yo[0]-yo[-1]
	#eq = np.hstack(( eq_x, eq_y, eq_z ))
	eq = np.hstack(( eq_nonhol, eq_x, eq_y, eq_z, eq_q1_boundary, eq_q2_boundary, eq_q3_boundary, eq_q4_boundary, eq_q5_boundary, eq_q6_boundary, eq_q1dot_boundary, eq_q2dot_boundary, eq_q3dot_boundary, eq_q4dot_boundary, eq_q5dot_boundary, eq_q6dot_boundary, eq_q1ddot_boundary, eq_q2ddot_boundary, eq_q3ddot_boundary, eq_q4ddot_boundary, eq_q5ddot_boundary, eq_q6ddot_boundary, eq_x_boundary, eq_y_boundary, eq_xdot_boundary, eq_ydot_boundary, eq_xddot_boundary, eq_yddot_boundary    ))


	f = 0.0001*(np.sum(q_1_ddot**2)+np.sum(q_2_ddot**2)+np.sum(q_3_ddot**2)+np.sum(q_4_ddot**2)+np.sum(q_5_ddot**2)+np.sum(q_6_ddot**2)+np.sum(x_base_vel**2)+np.sum(y_base_vel**2))+200*np.sum(eq**2)
	#f = np.sum(eq**2)
	return f

maxiter = 600

numdof_manipulator = 6
theta_min = np.array([-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi])
theta_max = np.array([np.pi, 180*(np.pi/180), np.pi, np.pi, np.pi, np.pi])
#print ("theta_min",theta_min)
tot_time = np.linspace(0.0, 4 , 100) 

omega = 2*np.pi/tot_time[-1]

x_center = 0.1
y_center = 0.1
z_center = 1.3

rad_x = 1.30
rad_y = 0.30
rad_z = 0.10

t = tot_time[-1]/len(tot_time)

xf_traj = x_center+rad_x*np.cos(omega*tot_time)
yf_traj = y_center+rad_y*np.sin(omega*tot_time)
zf_traj = z_center+rad_z*np.cos(omega*tot_time)

P, Pdot, Pddot = bernstein_coeff_order10_arbitinterval.bernstein_coeff_order10_new(10, tot_time[0], tot_time[-1], tot_time.reshape(len(tot_time),1))
#cloumn of P
nvar =  np.shape(P)[1]
#row of P
num =   np.shape(P)[0]

num_dof = 9
x_guess = np.ones(num_dof*nvar)	
obj_grad = grad(obj_fun)

A_lin = block_diag(P, P, P, P, P, P) ############### joint limits.
#print "A_lin",A_lin
A_lin = np.hstack(( A_lin, np.zeros(( np.shape(A_lin)[0], 3*nvar ))  ))
#print "A_lin_new",A_lin
b_lin_ub = np.hstack(( theta_max[0]*np.ones(num), theta_max[1]*np.ones(num), theta_max[2]*np.ones(num), theta_max[3]*np.ones(num), theta_max[4]*np.ones(num), theta_max[5]*np.ones(num)  ))
b_lin_lb = np.hstack(( theta_min[0]*np.ones(num), theta_min[1]*np.ones(num), theta_min[2]*np.ones(num), theta_min[3]*np.ones(num), theta_min[4]*np.ones(num), theta_min[5]*np.ones(num)  ))

linear_constraint = LinearConstraint(A_lin, b_lin_lb, b_lin_ub)


opts = {'maxiter':maxiter}

sol = minimize(obj_fun, method='SLSQP', x0=x_guess, jac=obj_grad, constraints=[linear_constraint], options = opts )
x_sol = sol['x']

c_q1 = x_sol[0:nvar]
#print "c_q1",c_q1
c_q2 = x_sol[nvar:2*nvar]
c_q3 = x_sol[2*nvar:3*nvar]
c_q4 = x_sol[3*nvar:4*nvar]
c_q5 = x_sol[4*nvar:5*nvar]
c_q6 = x_sol[5*nvar:6*nvar]
c_x =  x_sol[6*nvar:7*nvar]
c_y =  x_sol[7*nvar:8*nvar]
c_phi = x_sol[8*nvar:9*nvar]
#print c_phi


q_1 = np.dot(P, c_q1)
#print np.shape(q_1)
q_2 = np.dot(P, c_q2)
#print len(q2)
q_3 = np.dot(P, c_q3)
#print len(q3)
q_4 = np.dot(P, c_q4)
#print len(q4)
q_5 = np.dot(P, c_q5)
#print len(q5)
q_6 = np.dot(P, c_q6)
#print len(q6)

x_base = np.dot(P, c_x)
y_base = np.dot(P, c_y)
phi = np.dot(P, c_phi)


# f = open('jointsangles_new.csv', 'w')
# jointsangles = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
# for i in range(len(q_1)):
# 	jointsangles.writerow([q_1[i],q_2[i],q_3[i],q_4[i],q_5[i],q_6[i],phi[i],x_base[i],y_base[i]])
				
x_base = np.dot(P, c_x)
y_base = np.dot(P, c_y)
phi = np.dot(P, c_phi)

xdot_base = np.dot(Pdot, c_x)
ydot_base = np.dot(Pdot, c_y)

#vel_base = np.sqrt(xdot_base**2+ydot_base**2)
#f = open('x_vel.csv', 'w')
#writer_x = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#for i in range (len(xdot_base)):
#	writer_x.writerow([xdot_base[i]])

# vel_base = np.sign(xdot_base)*np.sqrt(xdot_base**2+ydot_base**2)
# #print vel_base
# f = open('vel_base.csv', 'w')
# writer_x = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
# for i in range (len(vel_base)):
# 	writer_x.writerow([vel_base[i]])




# phidot_base = np.dot(Pdot, c_phi)
# f = open('phi_vel.csv', 'w')
# writer_phi = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
# for i in range (len(phidot_base)):
# 	writer_phi.writerow([phidot_base[i]])



plt.figure(1)
plt.plot(q_1, '-r', linewidth = 3.0)
plt.plot(q_2, '-g', linewidth = 3.0)
plt.plot(q_3, '-b', linewidth = 3.0)
plt.plot(q_4, '-k', linewidth = 3.0)
plt.plot(q_5, '-c', linewidth = 3.0)
plt.plot(q_6, '-m', linewidth = 3.0)


#plt.figure(2)
#plt.plot(xf_traj, yf_traj, '-b')
#plt.plot(x_base, y_base, '-r')
#plt.axis('equal')
#xx = []
#plt.show()
#print type(xe)
#print np.shape(xe)



#xe = [x._value for x in xe]
#ye = [x._value for x in ye]
#ze = [x._value for x in ze]
#print xe[-1]
fig = plt.figure()
ax = fig.gca(projection='3d')
#ax.plot(xe, ye, ze, label='FK_Calculated Trajector')
ax.plot(xf_traj, yf_traj, zf_traj, label='Given Circular Trajectory')
ax.legend()
plt.show()
