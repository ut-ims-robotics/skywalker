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

min_limit = np.array([-180, -180, -180, -180, -180, -180])*np.pi/180
max_limit = np.array([ 180,  180,  180,  180, 180, 180])*np.pi/180 

theta_init = np.array([0,0,0,0,0,0])
# desired_pose = Pose()

# desired_pose.position.x = 0.817
# desired_pose.position.y = 0.234
# desired_pose.position.z = 0.063
# desired_pose.orientation.x = 0  
# print desired_pose.orientation.x
# desired_pose.orientation.y =   0.707
# desired_pose.orientation.z =   0.707
# desired_pose.orientation.w =   0

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

# tot_time = np.linspace(0.0, 4 , 100) 
# omega = 2*np.pi/tot_time[-1]
# x_center = 0.1
# y_center = 0.1
# z_center = 1.3

# rad_x = 1.30
# rad_y = 0.30
# rad_z = 0.10

# t = tot_time[-1]/len(tot_time)

# xf_traj = x_center+rad_x*np.cos(omega*tot_time)
# #print (xf_traj)
# yf_traj = y_center+rad_y*np.sin(omega*tot_time)
# zf_traj = z_center+rad_z*np.cos(omega*tot_time)



# theta_compute = skywalker_ik.ik(min_limit,max_limit,theta_init,x_desired,y_desired,z_desired,desired_rotation_mat)
# print theta_compute
# xe,ye, ze,x_quat,y_quat,z_quat,w_quat  = skywalker_fk.fk(theta_compute)
# print "xe",xe
# print "ye",ye
# print "ze",ze
# print "x_quat",x_quat
# print "y_quat",y_quat
# print "z_quat",z_quat
# print "w_quat",w_quat



#### URDF KDL####

from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.joint_kinematics import JointKinematics
import rospy
robot = Robot.from_parameter_server()
base_link = "base_link"
end_link = "tool0"
print "Root link: %s; end link: %s" % (base_link, end_link)
#js_kin = JointKinematics(robot, base_link, end_link)
#print "Joint velocities:", js_kin.get_joint_velocities()
kdl_kin = KDLKinematics(robot, base_link, end_link)
#n = np.array([0,0,0,0,0,0])
#q = kdl_kin.random_joint_angles()
#q = Kdl_kin.
#print "Random angles:", q
#pose = kdl_kin.forward(q)
#print "FK:", pose


my_data = np.genfromtxt('xyz.csv', delimiter=',')

x_end = my_data[:,0] ############# x position of the end effector
y_end = my_data[:,1] ########## Y position of the end effector
z_end = my_data[:,2] ##############3 z position of the end effector

############ quaternions

#x_quat = my_data[:,3]
#y_quat = my_data[:,4]
#z_quat = my_data[:,5]
#w_quat = my_data[:,6]
xe = np.zeros(len(x_end))
ye = np.zeros(len(x_end))
ze = np.zeros(len(x_end))
x_quaternion = np.zeros(len(x_end))
y_quaternion = np.zeros(len(x_end))
z_quaternion = np.zeros(len(x_end))
w_quaternion = np.zeros(len(x_end))


q = []
for i in range(len(x_end)):
	theta_compute = skywalker_ik.ik(min_limit,max_limit,theta_init,x_end[i],y_end[i],z_end[i])
	q.append(theta_compute)
	#print type(q)
	
	with open('theta_compute.csv', mode='w') as joint_angles:
		joint_angles = csv.writer(joint_angles, delimiter=',')
	        joint_angles.writerows(q)

	theta_init = theta_compute
	#print theta_init
	x_e,y_e, z_e,x_quat,y_quat,z_quat,w_quat,fk  = skywalker_fk.fk(theta_compute)
	xe[i] = x_e
	ye[i] = y_e	
	ze[i] = z_e
	x_quaternion[i] = x_quat
	y_quaternion[i] = y_quat
	z_quaternion[i] = z_quat
	w_quaternion[i] = w_quat
	print "fk",fk
	print type(fk)
	print np.shape(fk)
	q_new = kdl_kin.inverse(fk)
	print "IK (not necessarily the same):", q_new
	if q_new is not None:
	    pose_new = kdl_kin.forward(q_new)
	    print "FK on IK:", pose_new
	    print type(pose_new)
	    print np.shape(pose_new)
	    print "Error:", np.linalg.norm(pose_new * fk**-1 - np.mat(np.eye(4)))
	else:
	    print "IK failure"
	
data = open('end_pose.csv', 'w')
end_pose = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
for i in range(len(xe)):
	end_pose.writerow([xe[i],ye[i],ze[i],x_quaternion[i],y_quaternion[i],z_quaternion[i],w_quaternion[i]])
			


#print "xe",xe
#print "ye",ye
#print "ze",ze




fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(xe, ye, ze, color="blue", linewidth=2.5, label='FK_Calculated Trajector')
ax.plot(x_end, y_end, z_end, color="red", linewidth=1.5, label='Desired Circular Trajectory')
ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()


	





