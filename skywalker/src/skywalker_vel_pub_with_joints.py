import roslib
import rospy
import actionlib
from numpy import *

from trajectory_msgs.msg import *
from control_msgs.msg import *

if __name__ == '__main__':
  rospy.init_node('simple_trajectory')
  client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
  client.wait_for_server()
  print ("server ok")
  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint', 'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint']


  joint_angles = genfromtxt('joint_angles.csv', delimiter=',')
  q1 = joint_angles[:,0]
  print q1 
  q2 = joint_angles[:,1] 
  q3 = joint_angles[:,2] 
  q4 = joint_angles[:,3]
  q5 = joint_angles[:,4]
  q6 = joint_angles[:,5]
  for i in range(len(joint_angles)):
    goal.trajectory.points.append(JointTrajectoryPoint([q1[i],q2[i],q3[i],q4[i],q5[i],q6[i]], [0, 0, 0, 0, 0, 0], [], [], rospy.Duration((1+i)*(0.2))))
  
  
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(1.0)

  client.send_goal(goal)
  
  print "Trajectory exectuion"
  print "client.get_state()",client.get_state()
  print "i am here",client.wait_for_result()
  print "client.get_state()",client.get_state() 
  print "i am in loop"
  print "client.get_state()",client.get_state()


  	
