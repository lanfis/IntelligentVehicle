#include <ros/ros.h>
#include <ros/spinner.h>
#include <nodelet/loader.h>
#include <string>
#include "PLATFORM_CONTROL/platform.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "platform_control";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());
  ros::NodeHandle n;

  ROS_INFO("%s activating ok !", nodeName.c_str());
  Platform pc(n, 0);
  
  while (ros::ok()) 
  {
//    pc.run();
  }
  //ros::spin();
  
  ros::shutdown();
  return 0;
}

