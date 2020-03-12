#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <stdlib.h>
// Include standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


int main(int argc, char** argv){
  ros::init(argc, argv, "tf2_listener");

  ros::NodeHandle node;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  std::ofstream myfile;
  std::vector<double> p;
  myfile.open ("/home/usman/usman-ros/src/skywalker/skywalker/src/end_pos_pose.csv");
  while (node.ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform("base_link", "ee_link",ros::Time(0));
      
      std::cout<<transformStamped.transform.translation.x<<std::endl;
      std::cout<<transformStamped.transform.translation.y<<std::endl;
      std::cout<<transformStamped.transform.translation.z<<std::endl;
      std::cout<<transformStamped.transform.rotation.x<<std::endl;
      std::cout<<transformStamped.transform.rotation.y<<std::endl;
      std::cout<<transformStamped.transform.rotation.z<<std::endl;
      std::cout<<transformStamped.transform.rotation.w<<std::endl;

      myfile <<transformStamped.transform.translation.x<<","<<transformStamped.transform.translation.y<<","<<
              transformStamped.transform.translation.z<<","<<transformStamped.transform.rotation.x<<","<<
              transformStamped.transform.rotation.y<<","<<transformStamped.transform.rotation.z<<","<<
              transformStamped.transform.rotation.w <<"\n";  
          


    }
    
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    rate.sleep();
  }
  //myfile.close();
  return 0;
};
