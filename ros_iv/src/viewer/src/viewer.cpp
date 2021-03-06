#include <ros/ros.h>
#include <ros/spinner.h>
#include <string>
#include "VIEWER/viewer.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "viewer";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());

  ROS_INFO("%s activating ...", nodeName.c_str());
  Viewer v(0);
  
  while (ros::ok()) 
  {
    v.run();
  }
  
  ros::shutdown();
  return 0;
}
