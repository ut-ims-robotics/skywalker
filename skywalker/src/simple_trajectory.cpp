#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("arm_controller/follow_joint_trajectory", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
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

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("ur5e_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("ur5e_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("ur5e_elbow_joint");
    goal.trajectory.joint_names.push_back("ur5e_wrist_1_joint");
    goal.trajectory.joint_names.push_back("ur5e_wrist_2_joint");
    goal.trajectory.joint_names.push_back("ur5e_wrist_3_joint");
    
    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(6);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 0.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(6);
    for (size_t j = 0; j < 6; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(6);
    goal.trajectory.points[ind].positions[0] = -0.3;
    goal.trajectory.points[ind].positions[1] = 0.2;
    goal.trajectory.points[ind].positions[2] = -1.1;
    goal.trajectory.points[ind].positions[3] = -1.2;
    goal.trajectory.points[ind].positions[4] = 1.5;
    goal.trajectory.points[ind].positions[5] = -1.3;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(6);
    for (size_t j = 0; j < 6; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "skywalker_manipulator_driver");

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
  return 0;
}
