#include <ros/ros.h>
#include <ros/spinner.h>
#include <string>
#include "INTERFACE/interface.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "interface";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());

  ROS_INFO("%s activating ...", nodeName.c_str());
  Interface i(0);
  
  while (ros::ok()) 
  {
    i.run();
  }
  
  ros::shutdown();
  return 0;
}
