

from numpy import *
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
import skywalker_forward_kinem_franka_lecture
from transforms3d import *
import time
import csv




#my_data = genfromtxt('joint_states.csv', delimiter=',')
my_data = [0,0,0,0,0,0]
print(my_data)
#x_e_compute_i = zeros(len(my_data))
#y_e_compute_i = zeros(len(my_data))
#z_e_compute_i = zeros(len(my_data))

#data = zeros(len(my_data))
#data_ = []

#for i in range(len(my_data)):
		
x_e_compute, y_e_compute, z_e_compute, R_e_compute = skywalker_forward_kinem_franka_lecture.forward_kinem_franka(my_data)
x = x_e_compute
y = y_e_compute
z = z_e_compute
print(x_e_compute)
print(y_e_compute)
print(z_e_compute)
	
	#data = hstack([x_e_compute,y_e_compute,z_e_compute])
	#print("data ",data)
	#data_.append(data)
	
	
	#x_e_compute_i[i] = x_e_compute
	#y_e_compute_i[i] = y_e_compute
	#z_e_compute_i[i] = z_e_compute
	#print("x_e_compute_",x_e_compute_)
	#print("y_e_compute_",y_e_compute_)
	#print("z_e_compute_",z_e_compute_)
	
	
	#with open('pose.csv', mode='w') as pose:
		#pose = csv.writer(pose, delimiter=',')
	    	#pose.writerows(data_)
		
	
	

#fig = plt.figure()
#ax = fig.gca(projection='3d')

#ax.plot(x_e_compute, y_e_compute_i, z_e_compute_i, label='pose')
#ax.legend()
#ax.set_xlabel('X axis')
#ax.set_ylabel('Y axis')
#ax.set_zlabel('Z axis')

#plt.show()





