#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>
#include <cstring>
#include <sstream>
#include "MOTOR_SYSTEM/motor_system.h"



using namespace ros;
using namespace std;



int main(int argc, char** argv)
{  
  string nodeName = "motor_system";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());

  ROS_INFO("%s activating ...", nodeName.c_str());
  Motor_System ms(0);
  
  while (ros::ok()) 
  {
    ms.run();
  }
  
  ros::shutdown();
  return 0;
}
