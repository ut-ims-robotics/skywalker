from numpy import array
import numpy as np
from sympy import *



q_1, q_2, q_3, q_4, q_5, q_6, phi = symbols('q_1, q_2, q_3, q_4, q_5, q_6, phi')
# alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6 = symbols('alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6')
# a_1, a_2, a_3, a_4, a_5, a_6 = symbols('a_1, a_2, a_3, a_4, a_5, a_6') 
# d_1, d_2, d_3, d_4, d_5, d_6 = symbols('d_1, d_2, d_3, d_4, d_5, d_6')

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

R_e = T_fin[0:3, 0:3]

R_base =  Matrix([[cos(phi) ,sin(phi) ,0],[sin(phi), cos(phi) ,0], [0 ,0 ,1] ])

R_fin = R_base*R_e

tr_R = R_fin[0,0]+R_fin[1,1]+R_fin[2,2]
ax1 = 0.5*(R_fin[2,1]-R_fin[1,2])
ax2 = 0.5*(R_fin[0,2]-R_fin[2,0])
ax3 = 0.5*(R_fin[1,0]-R_fin[0,1])



x_e_l = T_fin[0,3]
y_e_l = T_fin[1,3]
z_e_l = T_fin[2,3]

print "tr_R =",tr_R
print "ax1 =", ax1
print "ax2 =", ax2
print "ax3 =", ax3



print 'x_e_l =', x_e_l
print 'y_e_l =', y_e_l
print 'z_e_l =', z_e_l
