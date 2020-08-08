import roslib
import rospy
import actionlib

from trajectory_msgs.msg import *
from control_msgs.msg import *

if __name__ == '__main__':
  rospy.init_node('simple_trajectory')
  client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
  client.wait_for_server()
  print ("server ok")
  goal = FollowJointTrajectoryGoal()

  goal.trajectory.joint_names = ['ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint', 'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint']

  goal.trajectory.points.append(JointTrajectoryPoint([6.59947412e-01, -6.48157891e-01, 2.88448884e-02, 6.19364004e-01, -9.26917071e-01, -8.29474869e-05], [0, 0, 0, 0, 0, 0], [], [], rospy.Duration(0.5)))

  goal.trajectory.points.append(JointTrajectoryPoint( [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [],[], rospy.Duration(1.0)))

  goal.trajectory.points.append(JointTrajectoryPoint([6.59947412e-01, -6.48157891e-01, 2.88448884e-02, 6.19364004e-01, -9.26917071e-01, -8.29474869e-05], [0, 0, 0, 0, 0, 0], [],[], rospy.Duration(1.5)))
  #goal.trajectory.points.append(JointTrajectoryPoint( [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [],[], rospy.Duration(2.0)))


  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(1.0)

  client.send_goal(goal)

  if (client.wait_for_result() ):
  	print "i am waiting for results"
  print "i am outside of while loop"
  #print client.get_state()
 # while(not client.is_finished):
#	print( "i am in loop")
 # print "i am done with waypoints" 
