#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from numpy import *

my_data = genfromtxt('/home/usman/usman-ros/src/skywalker/skywalker/src/intial_pose.csv', delimiter=',')

x = my_data[0,0]
print "x:", x
y = my_data[0,1]
print "y:", y
phi = my_data[0,2]
print "phi:", phi
t = 10

###### tetha_0
tetha_0 = arctan2((y),(x))
print "tetha_0",tetha_0
angular_vel_0 = tetha_0/t
#############

linear_d = sqrt((x*x)+(y*y))
print "linear_d",linear_d
linear_vel = linear_d/t

##### angular vel


angular_d =  phi- tetha_0
print "angular_d:", angular_d
angular_vel = angular_d/t
#print "angular_vel:", angular_vel
#######
def move():

    # Starts a new node
    rospy.init_node('skywalker_intial_pose', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=10)
    vel_msg = TwistStamped()
######################################################################################
    vel_msg.twist.linear.x = 0
    vel_msg.twist.linear.y = 0
    vel_msg.twist.linear.z = 0
    vel_msg.twist.angular.x = 0
    vel_msg.twist.angular.y = 0
    vel_msg.twist.angular.z = 0
    velocity_publisher.publish(vel_msg)



######################################################################################
##				1-step
######################################################################################
    ######### tetha_0 ########
    # Checking if our movement is CW or CCW
    vel_msg.twist.linear.x = 0
    vel_msg.twist.linear.y = 0
    vel_msg.twist.linear.z = 0
    vel_msg.twist.angular.x = 0
    vel_msg.twist.angular.y = 0
    if (angular_vel_0 < 0 ):
	    #CW
        vel_msg.twist.angular.z = -abs(angular_vel_0)
    else:
	    #CCW
        vel_msg.twist.angular.z = abs(angular_vel_0)
	# Setting the current time for distance calculus
    
    rospy.sleep(2)
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    #print "robot start"
    while(current_angle < abs(tetha_0)):
	    #print "robot is moving in angular direction"
	    velocity_publisher.publish(vel_msg)
	    t1 = rospy.Time.now().to_sec()
	    current_angle = abs(angular_vel_0)*(t1-t0)
    print "current_angle =", current_angle
    print "delta_t_angular=",(t1-t0)
    	#Forcing our robot to stop
    vel_msg.twist.angular.z = 0
    velocity_publisher.publish(vel_msg)
###################################################################################### 

##			2-step
######################################################################################   
        
    
    vel_msg.twist.linear.x = abs(linear_vel)
    vel_msg.twist.linear.y = 0
    vel_msg.twist.linear.z = 0
    vel_msg.twist.angular.x = 0
    vel_msg.twist.angular.y = 0
    vel_msg.twist.angular.z = 0
    
    rospy.sleep(2)
    #Setting the current time for distance calculus
    t2 = rospy.Time.now().to_sec()
    current_distance = 0

    #Loop to move the turtle in an specified distance
    while(current_distance < abs(linear_d)):
        #Publish the velocity
    #print "robot is moving in linear direction"
        velocity_publisher.publish(vel_msg)
        #Takes actual time to velocity calculus
        t3=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance= abs(linear_vel)*(t3-t2)
    print "current_distance =", current_distance
    print "delta_t_linear=",(t3-t2)
    #After the loop, stops the robot
    vel_msg.twist.linear.x = 0
    #Force the robot to stop
    velocity_publisher.publish(vel_msg)
    print "robot step 3"
######################################################################################


###			3-step
#####################################################
	# Checking if our movement is CW or CCW
    vel_msg.twist.linear.x = 0
    vel_msg.twist.linear.y = 0
    vel_msg.twist.linear.z = 0
    vel_msg.twist.angular.x = 0
    vel_msg.twist.angular.y = 0
    if (angular_vel < 0):
	    #CW
        vel_msg.twist.angular.z = -abs(angular_vel)
    else:
	    #CCW
        vel_msg.twist.angular.z = abs(angular_vel)
	# Setting the current time for distance calculus
    rospy.sleep(2)
    t4 = rospy.Time.now().to_sec()
    current_angle = 0
    #print "robot start"
    while(current_angle < abs(angular_d)):
	    #print "robot is moving in angular direction"
	    velocity_publisher.publish(vel_msg)
	    t5 = rospy.Time.now().to_sec()
	    current_angle = abs(angular_vel)*(t5-t4)
    print "current_angle =", current_angle
    print "delta_t_angular=",(t5-t4)
    	#Forcing our robot to stop
    vel_msg.twist.angular.z = 0
    velocity_publisher.publish(vel_msg)
######################################################################################       
if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
