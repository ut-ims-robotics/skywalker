import rospy
# import math
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from numpy import *

my_data = genfromtxt('/home/usman/skywalker/skywalker/src/elliptical_traj/elliptical_translation.csv', delimiter=',')

x = my_data[:,0]
print (len(x))
y = my_data[:,1] 
z = my_data[:,2]

if __name__ == '__main__':
    rospy.init_node('Given_Traj')
    marker_pub = rospy.Publisher('Given_traj_marker', Marker, queue_size=1000) 

    marker = Marker() 
    marker.id = 0
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.LINE_STRIP
    marker.ns = 'Given_Traj'
    marker.action = Marker.ADD
    marker.scale.x = 0.02
    marker.pose.orientation.w = 1.0
    marker.color.a = 1.0
    marker.color.r = 1.0
    
    #R = rospy.get_param('~tf_ee_pub_rate')
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
	i = 0
	if ( i == 0):
		for j in range(size(x)):	
			newpoint = Point()
			newpoint.x = x[j]
			newpoint.y = y[j]
			newpoint.z = z[j]    
			print newpoint.x
			marker.points.append(newpoint)
			i = 1       
    	marker_pub.publish(marker)
    	rate.sleep()

