#include <ros/ros.h>
#include <ros/spinner.h>
#include <string>
#include "KCF_TRACKER/kcf_tracker.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "kcf_tracker";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());

  ROS_INFO("%s activating ...", nodeName.c_str());
  KCF_Tracker kcft(0);
  
  while (ros::ok()) 
  {
    kcft.run();
  }
  
  ros::shutdown();
  return 0;
}
