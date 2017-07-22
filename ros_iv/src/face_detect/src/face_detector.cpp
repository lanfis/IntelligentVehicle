#include <ros/ros.h>
#include <ros/spinner.h>
#include <string>
#include "FACE_DETECTOR/face_detector.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "face_detector";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());

  ROS_INFO("%s activating ...", nodeName.c_str());
  Face_Detector fd(0);
  
  while (ros::ok()) 
  {
    fd.run();
  }
  
  ros::shutdown();
  return 0;
}
