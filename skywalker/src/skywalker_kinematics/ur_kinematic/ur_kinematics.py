#!/usr/bin/python2

## UR5/UR10 Inverse Kinematics - Ryan Keating Johns Hopkins University


# ***** lib
import numpy as np
from numpy import linalg


import cmath
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi
from geometry_msgs.msg import Pose
import tf.transformations as tf
import csv

global mat
mat=np.matrix


# ****** Coefficients ******


global d1, a2, a3, d4, d5, d6
d1 =  0.1625
a2 = -0.425
a3 = -0.3922
#a7 = 0.075
d4 =  0.1333
d5 =  0.0997
d6 =  0.0996

global d, a, alph

d = mat([0.1625, 0, 0, 0.1333, 0.0997, 0.0996]) #ur5e
#d = mat([0.089159, 0, 0, 0.10915, 0.09465, 0.0823]) #ur5
#d = mat([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])#ur10 mm
#a =mat([0 ,-0.425 ,-0.39225 ,0 ,0 ,0]) #ur5
a =mat([0 ,-0.425 ,-0.3922 ,0 ,0 ,0]) #ur5e
#a =mat([0 ,-0.612 ,-0.5723 ,0 ,0 ,0])#ur10 mm
#alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0 ])  #ur5
#alph = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ]) # ur10
alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0 ])  #ur5e

my_data = np.genfromtxt('end_pos_pose.csv', delimiter=',')

x = -my_data[:,0] ############# x position of the end effector
print("x_end",np.shape(x))
y = -my_data[:,1] ########## Y position of the end effector
z = my_data[:,2] ##############3 z position of the end effector

############ quaternions

x_quat = my_data[:,3]
y_quat = my_data[:,4]
z_quat = my_data[:,5]
w_quat = my_data[:,6]

desired_pos = Pose()



# ************************************************** FORWARD KINEMATICS

def AH( n,th,c  ):

  T_a = mat(np.identity(4), copy=False)
  T_a[0,3] = a[0,n-1]
  T_d = mat(np.identity(4), copy=False)
  T_d[2,3] = d[0,n-1]

  Rzt = mat([[cos(th[n-1,c]), -sin(th[n-1,c]), 0 ,0],
	         [sin(th[n-1,c]),  cos(th[n-1,c]), 0, 0],
	         [0,               0,              1, 0],
	         [0,               0,              0, 1]],copy=False)
      

  Rxa = mat([[1, 0,                 0,                  0],
			 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
			 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
			 [0, 0,                 0,                  1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa
	    

  return A_i

def HTrans(th,c ):  
  A_1=AH( 1,th,c  )
  A_2=AH( 2,th,c  )
  A_3=AH( 3,th,c  )
  A_4=AH( 4,th,c  )
  A_5=AH( 5,th,c  )
  A_6=AH( 6,th,c  )
      
  T_06=A_1*A_2*A_3*A_4*A_5*A_6

  return T_06

# ************************************************** INVERSE KINEMATICS 

def invKine(desired_pos):# T60
  th = mat(np.zeros((6, 8)))
  P_05 = (desired_pos * mat([0,0, -d6, 1]).T-mat([0,0,0,1 ]).T)
  
  # **** theta1 ****
  
  psi = atan2(P_05[2-1,0], P_05[1-1,0])
  phi = acos(d4 /sqrt(P_05[2-1,0]*P_05[2-1,0] + P_05[1-1,0]*P_05[1-1,0]))
  #The two solutions for theta1 correspond to the shoulder
  #being either left or right
  th[0, 0:4] = pi/2 + psi + phi
  th[0, 4:8] = pi/2 + psi - phi
  th = th.real
  
  
  # **** theta5 ****
  
  cl = [0, 4]# wrist up or down
  for i in range(0,len(cl)):
	      c = cl[i]
	      T_10 = linalg.inv(AH(1,th,c))
	      T_16 = T_10 * desired_pos
	      th[4, c:c+2] = + acos((T_16[2,3]-d4)/d6);
	      th[4, c+2:c+4] = - acos((T_16[2,3]-d4)/d6);

  th = th.real
  
  
  # **** theta6 ****
  # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
	      c = cl[i]
	      T_10 = linalg.inv(AH(1,th,c))
	      T_16 = linalg.inv( T_10 * desired_pos )
	      th[5, c:c+2] = atan2((-T_16[1,2]/sin(th[4, c])),(T_16[0,2]/sin(th[4, c])))
		  
  th = th.real
  

  # **** theta3 ****
  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
	      c = cl[i]
	      print ("c",c)
	      T_10 = linalg.inv(AH(1,th,c))
	      T_65 = AH( 6,th,c)
	      T_54 = AH( 5,th,c)
	      T_14 = ( T_10 * desired_pos) * linalg.inv(T_54 * T_65)
	      P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
	      t3 = cmath.acos((linalg.norm(P_13)**2 - a2**2 - a3**2 )/(2 * a2 * a3)) # norm ?
	      th[2, c] = t3.real
	      th[2, c+1] = -t3.real

  # **** theta2 and theta 4 ****

  cl = [0, 1, 2, 3, 4, 5, 6, 7]
  for i in range(0,len(cl)):
	      c = cl[i]
	      T_10 = linalg.inv(AH( 1,th,c ))
	      T_65 = linalg.inv(AH( 6,th,c))
	      T_54 = linalg.inv(AH( 5,th,c))
	      T_14 = (T_10 * desired_pos) * T_65 * T_54
	      P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
	      
	      # theta 2
	      th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a3* sin(th[2,c])/linalg.norm(P_13))
	      # theta 4
	      T_32 = linalg.inv(AH( 3,th,c))
	      T_21 = linalg.inv(AH( 2,th,c))
	      T_34 = T_32 * T_21 * T_14
	      th[3, c] = atan2(T_34[1,0], T_34[0,0])
  th = th.real
  

  return th
#print (desired_pos)

def ros2np(ros_pose):
    """Transform pose from ROS Pose format to np.array format.
    Args:
        ros_pose: A pose in ROS Pose format (type: Pose)
    Returns:
        An HTM (type: np.array).
    """

    # orientation
    np_pose = tf.quaternion_matrix([ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, ros_pose.orientation.w])
    
    # position
    np_pose[0][3] = ros_pose.position.x
    np_pose[1][3] = ros_pose.position.y
    np_pose[2][3] = ros_pose.position.z

    return np_pose
q1 = []
q2 = []
q3 = []
q4 = []
q5 = []
q6 = []
ik_column_1_q = []


for i in range(len(my_data)):

	desired_pos.position.x = x[i]
	desired_pos.position.y = y[i]
	desired_pos.position.z = z[i]
	desired_pos.orientation.x = x_quat[i]
	desired_pos.orientation.y = y_quat[i]
	desired_pos.orientation.z = z_quat[i]
	desired_pos.orientation.w = w_quat[i]
	#desired_pos = my_data[i]
	desired_pos_new = ros2np(desired_pos)
	#print(desired_pos_new)
	ik = invKine(desired_pos_new)
	ik_column_1 = ik[:,5].T
	#print (ik.T)
	ik_column_1_q.append(ik_column_1)

	q1.append(ik_column_1[0,0])
	#print(ik_column_1[0,0])
	q2.append(ik_column_1[0,1])
	#print(ik_column_1[0,1])
	q3.append(ik_column_1[0,2])

	#print(ik_column_1[0,2])
	q4.append(ik_column_1[0,3])
	#print(ik_column_1[0,3])
	q5.append(ik_column_1[0,4])
	#print(ik_column_1[0,4])
	q6.append(ik_column_1[0,5])
	#print(ik_column_1[0,5])
	q1_6 = [q1,q2,q3,q4,q5,q6]
	joint_angles = np.transpose(q1_6)
	#print(joint_angles)
	f = open('/home/usman/usman-ros/src/skywalker/skywalker/src/python-Universal-robot-kinematics/joint_angles.csv', 'w')
	writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	writer.writerows(joint_angles)



c = [0]
th = np.matrix([[-1.2407096154854536], [-1.7637555624092363], [-0.7515294201909519], [-0.13747956736272826], [0.7511499443730401], [-3.128427231784996]])
#print th
#fk = HTrans(th,c )
#print fk 
#ik = invKine(fk)
#print ik





