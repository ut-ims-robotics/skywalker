

from numpy import *
import csv
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D




#my_data = genfromtxt('end_pos_pose.csv', delimiter=',')
my_data = genfromtxt('joint_angles.csv', delimiter=',')

x_end = my_data[:,0] ############# x position of the end effector
y_end = my_data[:,1] ########## Y position of the end effector
z_end = my_data[:,2] ##############3 z position of the end effector

############ quaternions

x_quat = my_data[:,3]
y_quat = my_data[:,4]
z_quat = my_data[:,5]
w_quat = my_data[:,6]



fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x_end, y_end, z_end, label='parametric curve')
ax.legend()
plt.show()

