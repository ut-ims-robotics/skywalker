

from numpy import *
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
import skywalker_forward_kinem_franka_lecture
import skywalker_optim_ik_franka1
from transforms3d import *
import time
import csv
import tf.transformations as tf




#my_data = genfromtxt('end_pos_pose.csv', delimiter=',')

#x_end = my_data[:,0] ############# x position of the end effector
#y_end = my_data[:,1] ########## Y position of the end effector
#z_end = my_data[:,2] ##############3 z position of the end effector

############ quaternions

#x_quat_end = my_data[:,3]
#y_quat_end = my_data[:,4]
#z_quat_end = my_data[:,5]
#w_quat_end = my_data[:,6]

######## q_max and q_min values for each joint as given in the website 
#theta_min = array([-180, -180, -180, -180, -180, -180, -180 ])*pi/180
#theta_max = array([ 180,  180,  180,  180, 180, 180, 180])*pi/180 

#theta_actual = random.uniform(theta_min, theta_max, 7)
#print theta_actual
theta_actual = [0, 0 , 0 , 0 , 0 , 0]
print theta_actual
x_e_actual, y_e_actual, z_e_actual, R_e_actual = skywalker_forward_kinem_franka_lecture.forward_kinem_franka(theta_actual)
print x_e_actual 
print y_e_actual 
print z_e_actual
print R_e_actual

#theta_init = (theta_min+theta_max)

#x_e_compute_i = []
#y_e_compute_i = []
#z_e_compute_i = []
#q = []

#for i in range(len(my_data)):
#	x_e_actual = x_end[i]
#	y_e_actual = y_end[i]
#	z_e_actual = z_end[i]

	#x_q_e = x_quat_end[i]
	#y_q_e = y_quat_end[i]
	#z_q_e = z_quat_end[i]
	#w_q_e = w_quat_end[i]
	
	#qx = x_quat_end[i]
	#qy = y_quat_end[i]
	#qz = z_quat_end[i]
	#qw = w_quat_end[i]

	#R_e_actual = tf.quaternion_matrix([qx, qy, qz, qw])
	#R_e_actual = R_e_actual[0:3,0:3]
	#print(R_e_actual)
	#start = time.time()
	#theta_compute = skywalker_optim_ik_franka1.optim_ik_franka(theta_init, x_e_actual, y_e_actual, z_e_actual, R_e_actual, theta_min, theta_max)

	#q.append(theta_compute)
	#print q
	
	#with open('q_computed.csv', mode='w') as joint_angles:
		#joint_angles = csv.writer(joint_angles, delimiter=',')
	        #joint_angles.writerows(q)


	
	#print time.time()-start
	#theta_init = theta_compute
	#print theta_compute
	#print theta_actual

	#x_e_compute, y_e_compute, z_e_compute, R_e_compute = skywalker_forward_kinem_franka_lecture.forward_kinem_franka(theta_compute)

	#print x_e_actual, x_e_compute
	#print y_e_actual, y_e_compute
	#print z_e_actual, z_e_compute

	#euler_actual = euler.mat2euler(R_e_actual, 'sxyz')
	#euler_compute = euler.mat2euler(R_e_compute, 'sxyz')

	#print euler_actual
	#print euler_compute
	#x_e_compute_i.append(x_e_compute)
	#y_e_compute_i.append(y_e_compute)
	#z_e_compute_i.append(z_e_compute)
	#print("x_e_compute_i",x_e_compute_i)
	#print("y_e_compute_i",y_e_compute_i)
	#print("z_e_compute_i",z_e_compute_i)
	

#fig = plt.figure()
#ax = fig.gca(projection='3d')
#ax.plot(x_end, y_end, z_end, label='desiredparametric curve')
#ax.plot(x_e_compute_i, y_e_compute_i, z_e_compute_i, label='parametric curve')
#ax.legend()
#ax.set_xlabel('X axis')
#ax.set_ylabel('Y axis')
#ax.set_zlabel('Z axis')
#plt.show()





