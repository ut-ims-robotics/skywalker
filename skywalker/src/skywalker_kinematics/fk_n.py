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
c = [0]
th = np.matrix([[-1.2407096154854536], [-1.7637555624092363], [-0.7515294201909519], [-0.13747956736272826], [0.7511499443730401], [-3.128427231784996]])
print th
fk = HTrans(th,c )
print fk 







