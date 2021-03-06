/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman*/

#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "geometry_msgs/Pose.h"

#include <stdlib.h>
// Include standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_robot_model_and_robot_state");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using the RobotModel class is very easy. In
  // general, you will find that most higher-level components will
  // return a shared pointer to the RobotModel. You should always use
  // that when possible. In this example, we will start with such a
  // shared pointer and discuss only the basic API. You can have a
  // look at the actual code API for these classes to get more
  // information about how to use more features provided by these
  // classes.
  //
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/melodic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "panda_arm" of the Panda
  // robot.
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the Panda arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);    
  }

  // Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  /* Set one joint in the Panda arm outside its joint limit */
  joint_values[0] = 5;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  /* Check whether any joint is outside its joint limits */
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
  geometry_msgs::Pose pose;
  // Global vector pose( p ) .........vector to save position and orientation  
  std::vector<float> p;

  // open csv file for reading
      
  std::fstream file("/home/usman/skywalker/skywalker/src/elliptical_traj/ur5e_ee_pose.csv", std::ios::in);
  
  std::string line = "";    
  while (getline(file, line))
  {
    std::stringstream linestream(line);
    std::string value;
    //std::vector<double> p;
    // parse the line (comma separated joint states)
    while (getline(linestream, value, ','))
    {
      float val = std::strtof(value.c_str(), NULL);
      p.push_back(val);        
    }
  }

    std::ofstream myfile;
    myfile.open ("/home/usman/skywalker/skywalker/src/elliptical_traj/kdl_joint_angles.csv");
    // Jacobian
    std::ofstream jacobian_file;
    jacobian_file.open ("/home/usman/skywalker/skywalker/src/elliptical_traj/jacobian_kdl.csv");

  for (int i = 0 ; i < p.size()/7; i++)
    { 
      std::cout << " Print the pose"<< '\n' <<std::endl;
      std::cout << p[0+7*i] << std::endl;
      std::cout << p[1+7*i] << std::endl;
      std::cout << p[2+7*i] << std::endl;
      std::cout << p[3+7*i] << std::endl;
      std::cout << p[4+7*i] << std::endl;
      std::cout << p[5+7*i] << std::endl;
      std::cout << p[6+7*i] << '\n'<< std::endl;   
         
      pose.position.x = p[0+7*i];
      pose.position.y = p[1+7*i];
      pose.position.z = p[2+7*i];
      pose.orientation.x = p[3+7*i];
      pose.orientation.y = p[4+7*i];
      pose.orientation.z = p[5+7*i];   
      pose.orientation.w = p[6+7*i];

      // Inverse Kinematics
      // ^^^^^^^^^^^^^^^^^^
      // We can now solve inverse kinematics (IK) for the Panda robot.
      // To solve IK, we will need the following:
      //
      //  * The desired pose of the end-effector (by default, this is the last link in the "panda_arm" chain):
      //    end_effector_state that we computed in the step above.
      //  * The timeout: 0.1 s      
      double timeout = 0.5;
      bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);


      // Now, we can print out the IK solution (if found):
      if (found_ik)
      {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
          ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          myfile <<joint_values[i]<<",";
        }
        myfile <<"\n";
      }
      else
      {
        ROS_INFO("Did not find IK solution");
      }
    

      // Get the Jacobian
      // ^^^^^^^^^^^^^^^^
      // We can also get the Jacobian from the :moveit_core:`RobotState`.
      Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
      Eigen::MatrixXd jacobian;
      kinematic_state->getJacobian(joint_model_group,
                                  kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                  reference_point_position, jacobian);
                                  Eigen::MatrixXd j_t = jacobian.transpose();
      ROS_INFO_STREAM("Jacobian: \n" << jacobian  << "\n");
      ROS_INFO_STREAM("J_t: \n" << j_t << "\n");
      // END_TUTORIAL
      std::cout << "Dot product: " << (jacobian * j_t).determinant() << std::endl;
      jacobian_file <<(jacobian * j_t).determinant()<<","<< "\n";
    }
  myfile.close();
  jacobian_file.close();
  ros::shutdown();
  return 0;
}
