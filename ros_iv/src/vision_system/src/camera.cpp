#include <ros/ros.h>
#include <ros/spinner.h>
#include <string>
#include "VISION_SYSTEM/camera.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "camera";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());

  ROS_INFO("%s activating ...", nodeName.c_str());
  Camera cam(0);
  
  while (ros::ok()) 
  {
    cam.run();
  }
  
  ros::shutdown();
  return 0;
}
