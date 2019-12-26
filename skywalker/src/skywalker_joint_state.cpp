#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h> 
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv)
{
 // Initiate new ROS node named "skywalker_drive"
 ros::init(argc, argv, "skywalker_joint_state");
 ros::NodeHandle n;

 //Ceates the publisher, and tells it to publish
     //to the /cmd_vel topic, with a queue size of 1000
 ros::Publisher velocity_publisher = n.advertise<sensor_msgs::JointState>("mir/joint_states", 1000);
 ros::Rate loop_rate(1); //1 message per second

     //Sets up the random number generator
     srand(time(0));

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);
   while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
   {

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name.push_back("ur5e_shoulder_pan_joint");
    msg.name.push_back("ur5e_shoulder_lift_joint");
    msg.name.push_back("ur5e_elbow_joint");
    msg.name.push_back("ur5e_wrist_1_joint");
    msg.name.push_back("ur5e_wrist_2_joint");
    msg.name.push_back("ur5e_wrist_3_joint");
    std::vector<double> joint_values;
    msg.position.push_back(1);
    msg.position.push_back(0.2);
    msg.position.push_back(.1);
    msg.position.push_back(.1);
    msg.position.push_back(.5);
    msg.position.push_back(.6);      
    //set a random linear velocity in the x-axis
    //vel_msg.linear.x =(double)(rand() % 10 +1)/4.0;
    //vel_msg.linear.x =1;
    //vel_msg.linear.y =0;
    //vel_msg.linear.z =0;
    //set a random angular velocity in the y-axis
    //vel_msg.angular.x = 0;
    //vel_msg.angular.y = 0;
    //vel_msg.angular.z =(double)(rand() % 10 - 5)/2.0;

    //print the content of the message in the terminal
    // ROS_INFO("[Random Walk] linear.x = %.2f, angular.z=%.2f\n", vel_msg.linear.x, vel_msg.angular.z);

    //publish the message
    velocity_publisher.publish(msg);
       ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

   }
   return 0;
}

