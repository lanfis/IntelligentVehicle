#include <ros/ros.h>
#include <ros/spinner.h>
#include <nodelet/loader.h>
#include <string>
#include "MATRIX/matrix_node.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "matrix_node";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());
  ros::NodeHandle n;

  ROS_INFO("%s activating ok !", nodeName.c_str());
  Matrix_Node mn(n, 0);
  
  while (ros::ok()) 
  {
    mn.run();
  }
  //ros::spin();
  
  ros::shutdown();
  return 0;
}

/*
//#include "ros/ros.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "camera_nodelet");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "vision_system/camera_nodelet", remap, nargv);
  //ros::spin();
  
  ros::AsyncSpinner spinner(0);
  spinner.start();
  while(ros::ok())
  {
  }
    ros::shutdown();
  return 0;
  }
  */
