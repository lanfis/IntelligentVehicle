#include <ros/ros.h>
#include <ros/spinner.h>
#include <string>
#include "TNN/tnn_train_node.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "TNN_Train_Node";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());

  ROS_INFO("%s activating ...", nodeName.c_str());
  TNN_Train_Node tnn_train_node(0);
  /*
  while (ros::ok()) 
  {
    tnn_train_node.run();
  }
  */
  ros::shutdown();
  return 0;
}
