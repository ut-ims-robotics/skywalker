#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import csv
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener() #This will listen to the tf data later
    j_f = open('sim_elliptical.csv', 'w')

    ar_marker_pose = csv.writer(j_f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=100) 

    marker = Marker() 
    marker.id = 0
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.LINE_STRIP
    marker.ns = 'tf_listener_ee_node'
    marker.action = Marker.ADD
    marker.scale.x = 0.03
    marker.pose.orientation.w = 1.0
    marker.color.a = 0.2
    marker.color.g = 1.0
    
    #R = rospy.get_param('~tf_ee_pub_rate')
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('map', 'ur5e_tool0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
	print "trans: " , trans, "rot: ", rot
	print trans[1]
	#j_f = open('ar_marker_pose.csv', 'w')
	#ar_marker_pose = csv.writer(j_f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	#ar_marker_pose.writerow([trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3],'\n'])

	ar_marker_pose.writerow([trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]])
	#ar_marker_pose.writerow('\n')
	print(trans)
        newpoint = Point()
        newpoint.x = trans[0]
        newpoint.y = trans[1]
        newpoint.z = trans[2]

        if len(marker.points) > 100:
            marker.points.pop(0)        # To make the old trail disappear continuously

        marker.points.append(newpoint)
        
        marker_pub.publish(marker)

        rate.sleep()

