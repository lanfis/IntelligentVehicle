#include <ros/ros.h>
#include <ros/spinner.h>
#include <string>
#include "TRACKING_SYSTEM/tracking_system.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "tracking_system";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());

  ROS_INFO("%s activating ...", nodeName.c_str());
  Tracking_System ts(0);
  
  while (ros::ok()) 
  {
    ts.run();
  }
  
  ros::shutdown();
  return 0;
}
