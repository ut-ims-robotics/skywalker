from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import csv 
import numpy as np
#import math
import time
import skywalker_ik
import skywalker_fk
from geometry_msgs.msg import Pose
import tf.transformations as tf
xyz_min =  np.array([-100, -100])
xyz_max =  np.array([100, 100])
theta_min = np.array([-180, -180, -180, -180, -180, -180, -180])*np.pi/180
theta_max = np.array([ 180,  180,  180,  180, 180, 180, 180])*np.pi/180 
min_limit = np.concatenate ((theta_min,xyz_min))
max_limit = np.concatenate ((theta_max,xyz_max))
#print "max_limit",max_limit

theta_init = np.array([0,0,0,0,0,0,0,0,0])
# desired_pose = Pose()

# desired_pose.position.x = -1.971
# desired_pose.position.y = 1.193
# desired_pose.position.z = 1.389
# desired_pose.orientation.x =  0.504
# desired_pose.orientation.y =  -0.496
# desired_pose.orientation.z =  -0.496
# desired_pose.orientation.w =  0.504

# def desired_mat_2np(ros_pose):
#     """Transform pose from ROS Pose format to np.array format.

#     Args:
#         ros_pose: A pose in ROS Pose format (type: Pose)

#     Returns:
#         An HTM (type: np.array).
#     """

#     # orientation
#     np_pose = tf.quaternion_matrix([ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, ros_pose.orientation.w])
    
#     # position
#     np_pose[0][3] = ros_pose.position.x
#     np_pose[1][3] = ros_pose.position.y
#     np_pose[2][3] = ros_pose.position.z

#     return np_pose

# desired_mat = desired_mat_2np(desired_pose)
# #print (np.shape(desired_mat))
# #print (desired_mat)

# x_desired = desired_mat [0][3]
# #print x_desired
# y_desired = desired_mat [1][3]
# #print y_desired
# z_desired = desired_mat [2][3]
# #print z_desired
# desired_rotation_mat = desired_mat[0:3,0:3]
# #print (desired_rotation_mat)

# # ROS pose  ( pose of odom_comb frame )
# base_pose = Pose()

# # ROS position
# base_pose.position.x = 0.00015443975276
# x_base = base_pose.position.x
# base_pose.position.y = 1.66689143873e-06
# y_base = base_pose.position.y
# base_pose.position.z = 0.0
# z_base = base_pose.position.z

# base_pose.orientation.x = 0.0
# base_pose.orientation.y = 0.0
# base_pose.orientation.z = 0.00177472253077
# base_pose.orientation.w = 0.999998425179

# base_quaternion_mat = [base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w]
# #print("rotation",base_quaternion_mat)
# angle_x,angle_y,angle_z = tf.euler_from_quaternion(base_quaternion_mat)
# #print angle_x
# #print angle_y
# #print angle_z
# th_ = angle_z
# print th_

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
#print (xf_traj)
yf_traj = y_center+rad_y*np.sin(omega*tot_time)
zf_traj = z_center+rad_z*np.cos(omega*tot_time)



#theta_compute = skywalker_ik.ik(min_limit,max_limit,theta_init,x_desired,y_desired,z_desired,desired_rotation_mat)
#print theta_compute
xe = np.zeros(len(xf_traj))
ye = np.zeros(len(xf_traj))
ze = np.zeros(len(xf_traj))
q = []
for i in range(len(xf_traj)):
	theta_compute = skywalker_ik.ik(min_limit,max_limit,theta_init,xf_traj[i],yf_traj[i],zf_traj[i])
	q.append(theta_compute)
	#print type(q)
	
	with open('theta_compute.csv', mode='w') as joint_angles:
		joint_angles = csv.writer(joint_angles, delimiter=',')
	        joint_angles.writerows(q)

	theta_init = theta_compute
	#print theta_init
	x_e,y_e, z_e  = skywalker_fk.fk(theta_compute)
	xe[i] = x_e
	
	ye[i] = y_e	
	ze[i] = z_e
#print "xe",xe
#print "ye",ye
#print "ze",ze
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(xe, ye, ze, color="blue", linewidth=2.5, label='FK_Calculated Trajector')
ax.plot(xf_traj, yf_traj, zf_traj, color="red", linewidth=1.5, label='Desired Circular Trajectory')
ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()


	





