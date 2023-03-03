# Skywalker
This repository contains packages to control UR5e (manipulator) and MiR100 (non-holonomic mobile base), which it is mounted on.

## Setup
### Prerequisites
* Ubuntu 20.04
* ROS1 Noetic
* Moveit
* Navigation stack
### Installlation
1. Clone this repository into src folder of your catkin workspace
2. Make sure branch is scafld-devel
3. Clone the submodules with command
```git submodule update --init --recursive```
4. go to root folder of catkin workspace and run ```rosdep install --from-paths src --ignore-src -r -y```
5. build workspace with ```catkin_make``` or ```catkin build``` if catkin_tools package is installed

## How to run on the real robot

Skywalker is set up in a way, that network is managed by DNS server on MiR. This means that UR5 will get the IP from MiR. There is a chance that it might change in between the robot start ups. Beware. IP of MiR should remain the same. 

### If MiR and UR5 IPs have not been changed

Run ```roslaunch skywalker skywalker_real.launch ```. It is expexted that both MiR and UR5 use the network and DNS server from MiR.

### If IP differ

1. Find out UR5 IP by starting it, going to settings on the pendant, going into network options and looking, what IP was assigned.
2. Run ```roslaunch skywalker skywalker_real.launch robot_ip:=192.168.12.xxx```, where xxx are new last IP digits.
