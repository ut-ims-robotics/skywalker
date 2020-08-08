import time
import autograd.numpy as np   # Thinly-wrapped version of Numpy
from autograd import grad
import autograd
from scipy.linalg import block_diag
import matplotlib.pyplot as plt
import bernstein_coeff_order10_arbitinterval
from scipy.optimize import minimize
from scipy.linalg import block_diag
from scipy.optimize import LinearConstraint
from scipy.optimize import BFGS
from mpl_toolkits.mplot3d import Axes3D
import csv
import scipy.io as io



def obj_fun(c):
	global xe,ye,ze
	global f

	c_q1 = c[0:nvar]
	c_q2 = c[nvar:2*nvar]
	c_q3 = c[2*nvar:3*nvar]
	c_q4 = c[3*nvar:4*nvar]
	c_q5 = c[4*nvar:5*nvar]
	c_q6 = c[5*nvar:6*nvar]
	c_x =  c[6*nvar:7*nvar]
	c_y =  c[7*nvar:8*nvar]
	c_phi = c[8*nvar:9*nvar]

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
	
	xo = np.dot(P, c_x)
	yo = np.dot(P, c_y)
	phi_base = np.dot(P, c_phi)

	x_base_vel = np.dot(Pdot, c_x)
	y_base_vel = np.dot(Pdot, c_y)

	x_base_acc = np.dot(Pddot, c_x)
	y_base_acc = np.dot(Pddot, c_y)
				
	eq_nonhol = x_base_vel*np.sin(phi_base)-y_base_vel*np.cos(phi_base)
		

	x_e_l = -0.0996*((-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + ((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4))*np.sin(q_5) - 0.0997*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + 0.0997*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 0.3922*(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - 0.3922*(6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3) + 0.0996*(6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 1.0*np.sin(q_1))*np.cos(q_5) - 2.60237444818813e-17*np.sin(q_1)*np.sin(q_2) - 0.1333*np.sin(q_1) + 0.425*np.cos(q_1)*np.cos(q_2)
	y_e_l = -0.0996*(((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + ((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4))*np.sin(q_5) + 0.0997*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) - 0.0997*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) - 0.3922*(np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) - 0.3922*(-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3) + 0.0996*(-6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*np.cos(q_1))*np.cos(q_5) + 0.425*np.sin(q_1)*np.cos(q_2) + 2.60237444818813e-17*np.sin(q_2)*np.cos(q_1) + 0.1333*np.cos(q_1)
	z_e_l = -0.0996*((-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.sin(q_4) + (1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.cos(q_4))*np.sin(q_5) - 0.0997*(-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.cos(q_4) + 0.0997*(1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.sin(q_4) + 0.0996*(6.12323399573677e-17*(-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*(1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.sin(q_4) + 6.12323399573677e-17)*np.cos(q_5) - 0.3922*np.sin(q_2)*np.cos(q_3) - 0.425*np.sin(q_2) - 0.3922*np.sin(q_3)*np.cos(q_2) + 0.1625


	tr = (-(((-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + ((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4))*np.cos(q_5) + (6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 1.0*np.sin(q_1))*np.sin(q_5))*np.sin(q_6) + (-6.12323399573677e-17*((-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + ((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4))*np.sin(q_5) + 1.0*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 1.0*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) + 6.12323399573677e-17*(6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 1.0*np.sin(q_1))*np.cos(q_5) + 6.12323399573677e-17*np.sin(q_1))*np.cos(q_6))*np.sin(phi_base) + ((((-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + ((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4))*np.cos(q_5) + (6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 1.0*np.sin(q_1))*np.sin(q_5))*np.cos(q_6) + (-6.12323399573677e-17*((-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + ((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4))*np.sin(q_5) + 1.0*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 1.0*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) + 6.12323399573677e-17*(6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 1.0*np.sin(q_1))*np.cos(q_5) + 6.12323399573677e-17*np.sin(q_1))*np.sin(q_6))*np.cos(phi_base) + (-((((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + ((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4))*np.cos(q_5) + (-6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*np.cos(q_1))*np.sin(q_5))*np.sin(q_6) + (-6.12323399573677e-17*(((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + ((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4))*np.sin(q_5) - 1.0*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 1.0*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 6.12323399573677e-17*(-6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*np.cos(q_1))*np.cos(q_5) - 6.12323399573677e-17*np.cos(q_1))*np.cos(q_6))*np.cos(phi_base) + (((((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + ((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4))*np.cos(q_5) + (-6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*np.cos(q_1))*np.sin(q_5))*np.cos(q_6) + (-6.12323399573677e-17*(((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + ((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4))*np.sin(q_5) - 1.0*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 1.0*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 6.12323399573677e-17*(-6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*np.cos(q_1))*np.cos(q_5) - 6.12323399573677e-17*np.cos(q_1))*np.sin(q_6))*np.sin(phi_base) - 1.0*((-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.sin(q_4) + (1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.cos(q_4))*np.sin(q_5) - 6.12323399573677e-17*(-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.cos(q_4) + 6.12323399573677e-17*(1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.sin(q_4) + 1.0*(6.12323399573677e-17*(-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*(1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.sin(q_4) + 6.12323399573677e-17)*np.cos(q_5) + 2.29584502165847e-49

	ax1 = -0.5*(((-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.sin(q_4) + (1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.cos(q_4))*np.cos(q_5) + (6.12323399573677e-17*(-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*(1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.sin(q_4) + 6.12323399573677e-17)*np.sin(q_5))*np.sin(q_6) - 0.5*(-1.0*((-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + ((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4))*np.sin(q_5) - 6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) + 1.0*(6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 1.0*np.sin(q_1))*np.cos(q_5) - 3.74939945665464e-33*np.sin(q_1))*np.sin(phi_base) - 0.5*(-1.0*(((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + ((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4))*np.sin(q_5) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) - 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*(-6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*np.cos(q_1))*np.cos(q_5) + 3.74939945665464e-33*np.cos(q_1))*np.cos(phi_base) + 0.5*(-6.12323399573677e-17*((-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.sin(q_4) + (1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.cos(q_4))*np.sin(q_5) + 1.0*(-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.cos(q_4) - 1.0*(1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.sin(q_4) + 6.12323399573677e-17*(6.12323399573677e-17*(-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*(1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.sin(q_4) + 6.12323399573677e-17)*np.cos(q_5) - 3.74939945665464e-33)*np.cos(q_6)

	ax2 = -0.5*(((-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.sin(q_4) + (1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.cos(q_4))*np.cos(q_5) + (6.12323399573677e-17*(-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*(1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.sin(q_4) + 6.12323399573677e-17)*np.sin(q_5))*np.cos(q_6) + 0.5*(-1.0*((-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + ((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4))*np.sin(q_5) - 6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) + 1.0*(6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 1.0*np.sin(q_1))*np.cos(q_5) - 3.74939945665464e-33*np.sin(q_1))*np.cos(phi_base) + 0.5*(-1.0*(((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + ((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4))*np.sin(q_5) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) - 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*(-6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*np.cos(q_1))*np.cos(q_5) + 3.74939945665464e-33*np.cos(q_1))*np.sin(phi_base) - 0.5*(-6.12323399573677e-17*((-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.sin(q_4) + (1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.cos(q_4))*np.sin(q_5) + 1.0*(-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.cos(q_4) - 1.0*(1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.sin(q_4) + 6.12323399573677e-17*(6.12323399573677e-17*(-1.0*np.sin(q_2)*np.sin(q_3) + 1.0*np.cos(q_2)*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*(1.0*np.sin(q_2)*np.cos(q_3) + 1.0*np.sin(q_3)*np.cos(q_2))*np.sin(q_4) + 6.12323399573677e-17)*np.cos(q_5) - 3.74939945665464e-33)*np.sin(q_6)

	ax3 = -0.5*(-(((-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + ((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4))*np.cos(q_5) + (6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 1.0*np.sin(q_1))*np.sin(q_5))*np.sin(q_6) + (-6.12323399573677e-17*((-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + ((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4))*np.sin(q_5) + 1.0*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 1.0*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) + 6.12323399573677e-17*(6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 1.0*np.sin(q_1))*np.cos(q_5) + 6.12323399573677e-17*np.sin(q_1))*np.cos(q_6))*np.cos(phi_base) + 0.5*((((-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + ((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4))*np.cos(q_5) + (6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 1.0*np.sin(q_1))*np.sin(q_5))*np.cos(q_6) + (-6.12323399573677e-17*((-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + ((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4))*np.sin(q_5) + 1.0*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 1.0*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) + 6.12323399573677e-17*(6.12323399573677e-17*(-(6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) - 6.12323399573677e-17*((6.12323399573677e-17*np.sin(q_1)*np.sin(q_2) - np.cos(q_1)*np.cos(q_2))*np.cos(q_3) + (6.12323399573677e-17*np.sin(q_1)*np.cos(q_2) + np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4) - 1.0*np.sin(q_1))*np.cos(q_5) + 6.12323399573677e-17*np.sin(q_1))*np.sin(q_6))*np.sin(phi_base) - 0.5*(-((((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + ((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4))*np.cos(q_5) + (-6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*np.cos(q_1))*np.sin(q_5))*np.sin(q_6) + (-6.12323399573677e-17*(((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + ((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4))*np.sin(q_5) - 1.0*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 1.0*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 6.12323399573677e-17*(-6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*np.cos(q_1))*np.cos(q_5) - 6.12323399573677e-17*np.cos(q_1))*np.cos(q_6))*np.sin(phi_base) + 0.5*(((((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + ((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4))*np.cos(q_5) + (-6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*np.cos(q_1))*np.sin(q_5))*np.cos(q_6) + (-6.12323399573677e-17*(((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.cos(q_4) + ((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.sin(q_4))*np.sin(q_5) - 1.0*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 1.0*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 6.12323399573677e-17*(-6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.sin(q_3) + (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.cos(q_3))*np.sin(q_4) + 6.12323399573677e-17*((np.sin(q_1)*np.sin(q_2) - 6.12323399573677e-17*np.cos(q_1)*np.cos(q_2))*np.cos(q_3) - (-np.sin(q_1)*np.cos(q_2) - 6.12323399573677e-17*np.sin(q_2)*np.cos(q_1))*np.sin(q_3))*np.cos(q_4) + 1.0*np.cos(q_1))*np.cos(q_5) - 6.12323399573677e-17*np.cos(q_1))*np.sin(q_6))*np.cos(phi_base)


	xe = (np.cos(phi_base)*(x_e_l+xc)-np.sin(phi_base)*(y_e_l+yc)  ) + xo
	ye = (np.sin(phi_base)*(x_e_l+xc)+np.cos(phi_base)*(y_e_l+yc)  ) + yo
	ze = z_e_l+zc
	eq_x = xe-xf_traj
	eq_y = ye-yf_traj
	eq_z = ze-zf_traj
	tr_g = tr-tr_des
	#print "tr_g:", tr_g
	#print "tr:", tr
	ax1_g = ax1-ax1_des
	#print "ax1_g:", ax1_g 
	ax2_g = ax2-ax2_des
	#print "ax2_g:", ax2_g 
	ax3_g = ax3-ax3_des
	#print "ax3_g:", ax3_g

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
		
		

	eq = np.hstack(( tr_g,ax1_g, ax2_g, ax3_g, eq_nonhol, eq_x, eq_y, eq_z, eq_q1_boundary, eq_q2_boundary, eq_q3_boundary, eq_q4_boundary, eq_q5_boundary, eq_q6_boundary, eq_q1dot_boundary, eq_q2dot_boundary, eq_q3dot_boundary, eq_q4dot_boundary, eq_q5dot_boundary, eq_q6dot_boundary, eq_q1ddot_boundary, eq_q2ddot_boundary, eq_q3ddot_boundary, eq_q4ddot_boundary, eq_q5ddot_boundary, eq_q6ddot_boundary, eq_x_boundary, eq_y_boundary, eq_xdot_boundary, eq_ydot_boundary, eq_xddot_boundary, eq_yddot_boundary    ))
	
	joint_limit_1 = np.hstack((  q_1-theta_max[0]*np.ones(num_horizon), -q_1+theta_min[0]*np.ones(num_horizon)     ))
	joint_limit_2 = np.hstack((  q_2-theta_max[1]*np.ones(num_horizon), -q_2+theta_min[1]*np.ones(num_horizon)     ))
	joint_limit_3 = np.hstack((  q_3-theta_max[2]*np.ones(num_horizon), -q_3+theta_min[2]*np.ones(num_horizon)     ))
	joint_limit_4 = np.hstack((  q_4-theta_max[3]*np.ones(num_horizon), -q_4+theta_min[3]*np.ones(num_horizon)     ))
	joint_limit_5 = np.hstack((  q_5-theta_max[4]*np.ones(num_horizon), -q_5+theta_min[4]*np.ones(num_horizon)     ))
	joint_limit_6 = np.hstack((  q_6-theta_max[5]*np.ones(num_horizon), -q_6+theta_min[5]*np.ones(num_horizon)     ))
	
	joint_limit = np.hstack((  joint_limit_1, joint_limit_2, joint_limit_3, joint_limit_4, joint_limit_5, joint_limit_6         ))

	joint_constraints = np.log(1+np.exp(joint_limit) )	

	joint_constraints_cost = np.sum( joint_constraints )	
	
	f = 0.0001*(np.sum(q_1_ddot**2)+np.sum(q_2_ddot**2)+np.sum(q_3_ddot**2)+np.sum(q_4_ddot**2)+np.sum(q_5_ddot**2)+np.sum(q_6_ddot**2)+w_2*(np.sum(x_base_vel**2)+np.sum(y_base_vel**2)))+200*np.sum(eq**2)+0.001*joint_constraints_cost
	
	return f

maxiter = 600
numdof_manipulator = 6
theta_min = np.array([-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi])
theta_max = np.array([np.pi, np.pi, np.pi, np.pi, np.pi, np.pi])
################# base motion penalty
w_2 = 100

#############################33
## Ellipses not centered at the origin
## we add offsets to the x and y terms to translate (or "move") the ellipse to the correct location
## x = x_center +a*np.cos(t)
# y = y_center +b*np.cos(t)
# a is the radius along the x-axis 
# b is the radius along the y-axis
tot_time = np.linspace(0.0, 20 , 100) 
omega = 2*np.pi/tot_time[-1]
num_horizon = len(tot_time)

x_center = 0.1
y_center = 0.4
z_center = 1.5
#z_center = 1.4

###Results
rad_x = 0.8
rad_y = 0.0
rad_z = 0.3

xc = -0.26235
yc = 0.1
zc = 0.842

t = tot_time[-1]/len(tot_time)

xf_traj = x_center+rad_x*np.cos(omega*tot_time) ## semi-major axis 
#print "xf_traj:",xf_traj
yf_traj = y_center+rad_y*np.sin(omega*tot_time) ## semi-minor axis 
#print "yf_traj:",yf_traj
zf_traj = z_center+rad_z*np.sin(omega*tot_time)
#print "zf_traj:",zf_traj

R_des = np.array([[-1.0000000,  0.0000000,  0.0000000], [0.0000000,  0.0000000,  1.0000000], [0.0000000,  1.0000000,  0.0000000]])

tr_des = R_des[0,0]+R_des[1,1]+R_des[2,2]
ax1_des = 0.5*(R_des[2,1]-R_des[1,2])
ax2_des = 0.5*(R_des[0,2]-R_des[2,0])
ax3_des = 0.5*(R_des[1,0]-R_des[0,1])

print "ax3_des: ",ax3_des
tr_des_n = np.zeros(len(xf_traj))
#print "tr_des_n:",type(tr_des_n)
ax1_des_n = np.zeros(len(xf_traj))
ax2_des_n = np.zeros(len(xf_traj))
ax3_des_n = np.zeros(len(xf_traj))
for i in range(len(xf_traj)):
	
	tr_des_n[i] = tr_des
	ax1_des_n[i]= ax1_des
	ax2_des_n[i]= ax2_des
	ax3_des_n[i]= ax3_des

print "tr_des_n: ",tr_des_n
P, Pdot, Pddot = bernstein_coeff_order10_arbitinterval.bernstein_coeff_order10_new(10, tot_time[0], tot_time[-1], tot_time.reshape(len(tot_time),1))
nvar =  np.shape(P)[1]
num =   np.shape(P)[0]

num_dof = 9
x_guess = np.zeros(num_dof*nvar)
obj_grad = grad(obj_fun)

A_lin = block_diag(P, P, P, P, P, P)
A_lin = np.hstack(( A_lin, np.zeros(( np.shape(A_lin)[0], 3*nvar ))  ))

b_lin_ub = np.hstack(( theta_max[0]*np.ones(num), theta_max[1]*np.ones(num), theta_max[2]*np.ones(num), theta_max[3]*np.ones(num), theta_max[4]*np.ones(num), theta_max[5]*np.ones(num)  ))
b_lin_lb = np.hstack(( theta_min[0]*np.ones(num), theta_min[1]*np.ones(num), theta_min[2]*np.ones(num), theta_min[3]*np.ones(num), theta_min[4]*np.ones(num), theta_min[5]*np.ones(num)  ))

linear_constraint = LinearConstraint(A_lin, b_lin_lb, b_lin_ub)

opts = {'maxiter':maxiter}

np.cost = []   
def log_cost(x):
  np.cost.append(f) 
start = time.time()
sol = minimize(obj_fun, method='SLSQP', x0=x_guess, jac=obj_grad, constraints=[linear_constraint], options = opts, callback=log_cost )
print time.time()-start
x_sol = sol['x']	
###

c_q1 = x_sol[0:nvar]
c_q2 = x_sol[nvar:2*nvar]
c_q3 = x_sol[2*nvar:3*nvar]
c_q4 = x_sol[3*nvar:4*nvar]
c_q5 = x_sol[4*nvar:5*nvar]
c_q6 = x_sol[5*nvar:6*nvar]
c_x =  x_sol[6*nvar:7*nvar]
c_y =  x_sol[7*nvar:8*nvar]
c_phi = x_sol[8*nvar:9*nvar]


q1 = np.dot(P, c_q1)
q2 = np.dot(P, c_q2)
q3 = np.dot(P, c_q3)
q4 = np.dot(P, c_q4)
q5 = np.dot(P, c_q5)
q6 = np.dot(P, c_q6)

x_base = np.dot(P, c_x)
y_base = np.dot(P, c_y)
phi_base = np.dot(P, c_phi)

xdot_base = np.dot(Pdot, c_x)
ydot_base = np.dot(Pdot, c_y)
phidot_base = np.dot(Pdot, c_phi)


##############################################
j_f = open('joint_angles.csv', 'w')
jointsangles = csv.writer(j_f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
for i in range(len(q1)):
	jointsangles.writerow([q1[i],q2[i],q3[i],q4[i],q5[i],q6[i]])

vel_base = np.sign(xdot_base)*np.sqrt(xdot_base**2+ydot_base**2)
f = open('x_vel.csv', 'w')
#f = open('xyphi.csv', 'w')
writer_x = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
for i in range (len(vel_base)):
	writer_x.writerow([vel_base[i]])
	#writer_x.writerow([x_base[i],y_base[i],phi_base[i]])
f1 = open('phi_vel.csv', 'w')
writer_phi = csv.writer(f1, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
for i in range (len(phidot_base)):
 	writer_phi.writerow([phidot_base[i]])

			
########## Desired Tajectory 
traj = open('translation.csv', 'w')
traj_p = csv.writer(traj, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
for i in range(len(xf_traj)):
	traj_p.writerow([xf_traj[i],yf_traj[i],zf_traj[i]])


init_pose = open('intial_pose.csv', 'w')
init_pose_1 = csv.writer(init_pose, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
for i in range(len(xf_traj)):
	init_pose_1.writerow([x_base[i],y_base[i],phi_base[i]])

##########

#xe = [x._value for x in xe]
#ye = [x._value for x in ye]
#ze = [x._value for x in ze]



# plt.figure(1)
# plt.plot(q1, '-r', linewidth = 3.0)
# plt.plot(q2, '-g', linewidth = 3.0)
# plt.plot(q3, '-b', linewidth = 3.0)
# plt.plot(q4, '-k', linewidth = 3.0)
# plt.plot(q5, '-c', linewidth = 3.0)
# plt.plot(q6, '-m', linewidth = 3.0)
#print (f_1)
#np.cost = [x._value for x in np.cost]
#print np.cost


print type (xe)
print "xe:",xe
print "ye:",ye
print "ze:",ze
print "x_base:",x_base 
print "y_base:",y_base
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(xe, ye, ze, color="blue", linewidth=2.5, label='Trajector')
ax.plot(xf_traj, yf_traj, zf_traj, color="red", linewidth=1.5, label='Desired Trajectory')
ax.plot(x_base, y_base)
ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()





















