

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  move_group.clearPathConstraints();

  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

  //geometry_msgs::Pose target_pose3 = move_group.getCurrentPose("tool0").pose;
  std::cout << "The end effector link is "<<move_group.getEndEffectorLink() << std::endl;
  std::cout << "The planning frame is "<< move_group.getPlanningFrame() << std::endl;

  std::cout << target_pose3.position.x<<", "<< target_pose3.position.y<<", "<< target_pose3.position.z<<", "<< target_pose3.orientation.x<<", "
  << target_pose3.orientation.y<<", "<< target_pose3.orientation.z<<", "<< target_pose3.orientation.w<<", "<<std::endl;

  ros::shutdown();
  return 0;
}
