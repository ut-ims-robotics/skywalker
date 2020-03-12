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
alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2,0 ])  #ur5e




def np2ros(np_pose):
    """Transform pose from np.array format to ROS Pose format.

    Args:
        np_pose: A pose in np.array format (type: np.array)

    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = Pose()

    # ROS position
    ros_pose.position.x = np_pose[0, 3]
    ros_pose.position.y = np_pose[1, 3]
    ros_pose.position.z = np_pose[2, 3]

    # ROS orientation 
    np_q = tf.quaternion_from_matrix(np_pose)
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]

    return ros_pose
def AH( n,th ):

  T_a = mat(np.identity(4), copy=False)
  T_a[0,3] = a[0,n-1]
  T_d = mat(np.identity(4), copy=False)
  T_d[2,3] = d[0,n-1]


  Rzt = mat([[cos(th), -sin(th), 0 ,0],
	         [sin(th),  cos(th), 0, 0],
	         [0,               0,              1, 0],
	         [0,               0,              0, 1]],copy=False)
      
  Rxa = mat([[1, 0,                 0,                  0],
			 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
			 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
			 [0, 0,                 0,                  1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa
  
  #print A_i
	    

  return A_i

def HTrans(th):  
  A_1=AH( 1,th[0]  )
  A_2=AH( 2,th[1]  )
  A_3=AH( 3,th[2]  )
  A_4=AH( 4,th[3]  )
  A_5=AH( 5,th[4]  )
  A_6=AH( 6,th[5]  )
  cc = np.array(([-1 ,0 ,0,0],[0 ,-1 ,0,0],[0 ,0 ,1,0],[0 ,0 ,0,1]))
  #print(cc)
      
  T_06=cc *A_1*A_2*A_3*A_4*A_5*A_6
  #T_06=A_1*A_2*A_3*A_4*A_5*A_6

  return T_06


th  = [ 4.68662168e-01, -2.40242884e+00, -2.56349139e-03,  2.38086852e+00,
 -1.67363939e+00,  2.76213585e+00]

fk = HTrans(th)
fk_ros = np2ros(fk)
x_e = fk_ros.position.x + -0.26235
y_e = fk_ros.position.y + 0.1
z_e = fk_ros.position.z + 0.842
xyz_e  = np.array(([x_e],[y_e],[z_e]))
#print xyz_e
x_c = -0.0007199046876
y_c = -5.76686226703e-07
z_c = 0.0
xyz_c  = np.array(([x_c],[y_c],[z_c]))
#print xyz_c
th_ = -0.00251688112362
R_  = np.array(([cos(th_) ,-sin(th_),0],[sin(th_) ,cos(th_),0],[0 ,0 ,1]))
#print R_

xyz_b = np.dot(R_,xyz_e) + xyz_c
print xyz_b



#print(fk_ros)





