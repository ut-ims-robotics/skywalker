# Skywalker
The repository contains an optimization-based motion planning algorithm for UR5e (manipulator) mounted on a MiR(non-holonomic mobile base) and its ROS interface.

## Setup
* Ubuntu 18.04 for running ROS Melodic Distribution
* Clone the respository into to your catkin_ws and build it.
## Gazebo Simulation Implementation
To implement a desired end-effector trajectory on the mobile manipualtor( skywalker) in Gazebo.

open a new terminal: <br/>
``` roslaunch skywalker skywalker_empty_world.launch ```

To set initial pose, which can be done through the commands below:
<br />
``` rosrun skywalker intial_mobile_base_pose.py ```
<br />
``` rosrun skywalker sim_arm_joint_traj.cpp ```
<br/>

After, To implement complete trajectory 
<br />
``` rosrun skywalker sim_mm_traj.cpp ```
<br />
## Real Hardware Implementation
To implement a desired end-effector trajectory on the actual mobile manipualtor hardware

### To start ROS drivers on MiR and UR5e 
1. In the first terminal: <br/>
``` roslaunch mir_driver mir.launch ```
2. In another terminal: <br />
``` roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.x.xxx ```

To set initial pose
<br />
``` rosrun skywalker intial_mobile_base_pose.py ```
<br />
``` rosrun skywalker real_arm_joint_traj.cpp ```
<br/>

After, To implement complete trajectory 
<br />
``` rosrun skywalker real_mm_traj.cpp ```
<br />
## AR tag tracking library
### Install
<br />
``` sudo apt-get install ros-melodic-ar-track-alvar ```
<br />
``` roslaunch skywalker ar_tracker_side.launch ```
