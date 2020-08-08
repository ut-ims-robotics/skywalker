#!/usr/bin/env python
import rospy
import csv
from sensor_msgs.msg import JointState


def joint_states_callback(message):
    # filter out joint0 position:
    #for i,name in enumerate(message.name):
        #if name == "joint0":
    pos = message.position
    writer.writerow([pos[4],pos[3],pos[2],pos[5],pos[6],pos[7]])
    print("joint_state",pos)
    
    
     
    return

if __name__ == '__main__':

    f = open('/home/usman/usman-ros/src/skywalker/skywalker/src/joint_states_fk.csv', 'w')
    writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    rospy.init_node("skywalker_joint_State")
    rospy.Subscriber("/skywalker/joint_states", JointState, joint_states_callback, queue_size=1)
    rospy.spin()
