#!/usr/bin/env python
import rospy
import pandas as pd
import numpy as np
import scipy.io as sc
from sensor_msgs.msg import JointState
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import csv
joint0 = [1,2]
joint1= [1,2]
joint2= [1,2]
joint3 = [1,2]
joint4= [1,2]
joint5= [1,2]
joint6 = [1,2]
poses  = []


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True,  disable_signals=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    group.execute(plan, wait=True)

  def callback(self,n):

    # joint0.append(data.position[0])
    # joint1.append(data.position[1])
    # joint2.append(data.position[2])
    # joint3.append(data.position[3])
    # joint4.append(data.position[4])
    # joint5.append(data.position[5])
    # joint6.append(data.position[6])
    # print (data.position[0])
    x = group.get_current_pose().pose
    writer.writerow([x.position.x, x.position.y, x.position.z, x.orientation.x,
      x.orientation.y, x.orientation.z, x.orientation.w])
    print "I am in the call back"
if __name__ == '__main__':    
  tutorial = MoveGroupPythonIntefaceTutorial()
  counter =10
  f = open('/home/usman/usman-ros/src/skywalker/skywalker/src/joint_positions.csv', 'w')
  writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  try:
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    rospy.Subscriber("/joint_states", JointState, tutorial.callback)
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    rospy.spin()
      
  except KeyboardInterrupt:
    print "The program has been terminated!"
    f.close()


# print ('here I am')
# nx = np.array((joint0,joint1,joint2,joint3,joint4,joint5,joint6))
# adict = {}
# adict['nx'] = nx
# sc.savemat('file.mat',adict)
# pd.DataFrame({'joint0': joint0, 'joint1': joint1, 'joint2':joint2, 'joint3':joint3, 'joint4':joint4, 'joint5':joint5, 'joint6':joint6}).to_csv('file.csv', index=False)

