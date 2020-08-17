#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/Twist.h"
#include <stdlib.h>
// Include standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class skywalker_manipulator
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  skywalker_manipulator() 
  {
    // tell the action client that we want to spin a thread by default
    //traj_client_ = new TrajClient("scaled_pos_traj_controller/follow_joint_trajectory", true);
    traj_client_ = new TrajClient("arm_controller/follow_joint_trajectory", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~skywalker_manipulator()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with i-th waypoints, used as an example
  /*! Note that this trajectory contains i-th waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;
    // Global vector q .........vector to save joint values 
    std::vector<float> q;

    // open csv file for reading
        
    std::fstream file("/home/usman/skywalker/skywalker/src/elliptical_traj/joint_angles.csv", std::ios::in);
   
    std::string line = "";    
    while (getline(file, line))
    {
      std::stringstream linestream(line);
      std::string value;
      //std::vector<double> q;
      // parse the line (comma separated joint states)
      while (getline(linestream, value, ','))
      {
        float val = std::strtof(value.c_str(), NULL);
        q.push_back(val);        
      }
    }


    //The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("ur5e_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("ur5e_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("ur5e_elbow_joint");
    goal.trajectory.joint_names.push_back("ur5e_wrist_1_joint");
    goal.trajectory.joint_names.push_back("ur5e_wrist_2_joint");
    goal.trajectory.joint_names.push_back("ur5e_wrist_3_joint");

    // We will have [q.size()/6] waypoints in this goal trajectory
    goal.trajectory.points.resize(q.size()/6);

    //cout<<"q_size::"<<q.size()/6;

    for (int i = 0 ; i < q.size()/6; i++)
    { 
      std::cout << " Print the Joints values q1 to q6"<< '\n' <<std::endl;
      std::cout << q[0+6*i] << std::endl;
      std::cout << q[1+6*i] << std::endl;
      std::cout << q[2+6*i] << std::endl;
      std::cout << q[3+6*i] << std::endl;
      std::cout << q[4+6*i] << std::endl;
      std::cout << q[5+6*i] << '\n'<< std::endl;        
 
      // // i-th trajectory point
      // // Positions
        
      goal.trajectory.points[i].positions.resize(6);
      goal.trajectory.points[i].positions[0] = q[0+6*i];
      goal.trajectory.points[i].positions[1] = q[1+6*i];
      goal.trajectory.points[i].positions[2] = q[2+6*i];
      goal.trajectory.points[i].positions[3] = q[3+6*i];
      goal.trajectory.points[i].positions[4] = q[4+6*i];
      goal.trajectory.points[i].positions[5] = q[5+6*i];
      // Velocities
      goal.trajectory.points[i].velocities.resize(6);
      for (size_t j = 0; j < 6; ++j)
      {
        goal.trajectory.points[i].velocities[j] = 0.0;
      }
      //To be reached  [(i+1.0)*(0.1)] seconds after starting along the trajectory
      goal.trajectory.points[i].time_from_start = ros::Duration((i+1.0)*(0.2));
           
    }  

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};
int count = 0;

int main(int argc, char **argv)
{
  // Initiate new ROS node named "skywalker_drive"
  ros::init(argc, argv, "skywalker_simple_drive");
  ros::NodeHandle n;

  //Ceates the publisher, and tells it to publish
  //to the /cmd_vel topic, with a queue size of 1000
  // ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  // ros::Rate loop_rate(1); //1 message per second

  //Sets up the random number generator
  //srand(time(0));

  //Sets the loop to publish at a rate of 10Hz
  ros::Rate rate(10);
  skywalker_manipulator arm;
  
  //while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
  //{    
      
     
    // geometry_msgs::Twist vel_msg;
    // //set a random linear velocity in the x-axis
    // //vel_msg.linear.x =(double)(rand() % 10 +1)/4.0;
    
    // if (count % 30 < 15)
    // {
    //   vel_msg.linear.x = 0.4;
    //   count = count + 1;
      
    // }
    // else
    // {
    //   vel_msg.linear.x = -0.4;
    //   count = count + 1;
      
      
    // }
    //std::cout<<"counter"<<count<<std::endl;
       
    //set a random angular velocity in the z-axis
    //vel_msg.angular.z =(double)(rand() % 10 - 5)/2.0;
    //vel_msg.angular.z = 0.2;

    //print the content of the message in the terminal
    //ROS_INFO("[Random Walk] linear.x = %.2f, angular.z=%.2f\n", vel_msg.linear.x, vel_msg.angular.z);
    //ROS_INFO("[Random Walk] linear.x = %.2f\n", vel_msg.linear.x);
    //publish the message

    // Wait for trajectory completion          
    if(!arm.getState().isDone())
    {
      
      //velocity_publisher.publish(vel_msg);
      //std::cout<<"Trying to complete trajectory "<<'\n'<<std::endl;
      //ROS_INFO(" Trying to complete trajectory: linear.x = %.2f\n", vel_msg.linear.x);
      
    }
    else
    {
      // Start the trajectory
      arm.startTrajectory(arm.armExtensionTrajectory());
      //velocity_publisher.publish(vel_msg);
      //ROS_INFO("[Random Walk] linear.x = %.2f\n", vel_msg.linear.x);
    }
    
  //ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
  //}
  
  return 0;
}








