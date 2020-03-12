

from numpy import *

def forward_kinem_franka(q):



	alpha1 = pi/2
	alpha2 = 0.0
	alpha3 = 0.0
	alpha4 = pi/2
	alpha5 = -pi/2
	alpha6 =  0.0

	d1 = 0.1625
	d2 = 0.0				
	d3 = 0.0
	d4 = 0.1333
	d5 = 0.0997
	d6 = 0.0996
	
	
	

	a1 = 0.0
	a2 = -0.425
	a3 = -0.3922
	a4 = 0.0
	a5 = 0.0
	a6 = 0.0
	
		

	q1 = q[0]
	q2 = q[1]
	q3 = q[2]
	q4 = q[3]
	q5 = q[4]
	q6 = q[5]
	

	T1 = array([[cos(q1), -sin(q1)*cos(alpha1), sin(q1)*sin(alpha1), a1*cos(q1)], [ sin(q1), cos(q1)*cos(alpha1), -cos(q1)*sin(alpha1), a1*sin(q1)], [0.0, sin(alpha1), cos(alpha1), d1 ], [0.0, 0.0, 0.0, 1.0 ]])

	T2 = array([[cos(q2), -sin(q2)*cos(alpha2), sin(q2)*sin(alpha2), a2*cos(q2)], [ sin(q2), cos(q2)*cos(alpha2), -cos(q2)*sin(alpha2), a2*sin(q2)], [0.0, sin(alpha2), cos(alpha2), d2 ], [0.0, 0.0, 0.0, 1.0 ]])

	T3 = array([[cos(q3), -sin(q3)*cos(alpha3), sin(q3)*sin(alpha3), a3*cos(q3)], [ sin(q3), cos(q3)*cos(alpha3), -cos(q3)*sin(alpha3), a3*sin(q3)], [0.0, sin(alpha3), cos(alpha3), d3 ], [0.0, 0.0, 0.0, 1.0 ]])

	T4 = array([[cos(q4), -sin(q4)*cos(alpha4), sin(q4)*sin(alpha4), a4*cos(q4)], [ sin(q4), cos(q4)*cos(alpha4), -cos(q4)*sin(alpha4), a4*sin(q4)], [0.0, sin(alpha4), cos(alpha4), d4 ], [0.0, 0.0, 0.0, 1.0 ]])

	T5 = array([[cos(q5), -sin(q5)*cos(alpha5), sin(q5)*sin(alpha5), a5*cos(q5)], [ sin(q5), cos(q5)*cos(alpha5), -cos(q5)*sin(alpha5), a5*sin(q5)], [0.0, sin(alpha5), cos(alpha5), d5 ], [0.0, 0.0, 0.0, 1.0 ]])

	T6 = array([[cos(q6), -sin(q6)*cos(alpha6), sin(q6)*sin(alpha6), a6*cos(q6)], [ sin(q6), cos(q6)*cos(alpha6), -cos(q6)*sin(alpha6), a6*sin(q6)], [0.0, sin(alpha6), cos(alpha6), d6 ], [0.0, 0.0, 0.0, 1.0 ]])


	T_fin = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
	
	R1 = T1[0:3, 0:3]
	R2 = T2[0:3, 0:3]
	R3 = T3[0:3, 0:3]
	R4 = T4[0:3, 0:3]
	R5 = T5[0:3, 0:3]
	R6 = T6[0:3, 0:3]
	#R7 = T7[0:3, 0:3]

	
	
		
	#vec_x = T_fin[0][3]
	#vec_y = T_fin[1][3]
	#vec_z = T_fin[2][3]

	vec_x = T_fin[0][3]-0.26235
	vec_y = T_fin[1][3]+0.1
	vec_z = T_fin[2][3]+0.842
	
	

	R_fin = T_fin[0:3, 0:3]

	trace_R_fin = trace(R_fin)
	

	return vec_x, vec_y, vec_z, R_fin



	





