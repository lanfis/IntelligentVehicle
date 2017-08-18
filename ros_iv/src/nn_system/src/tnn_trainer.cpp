#include <ros/ros.h>
#include <ros/spinner.h>
#include <nodelet/loader.h>
#include <string>
#include "TNN/tnn_trainer.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "nn_system";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());
  ros::NodeHandle n;

  ROS_INFO("%s activating ok !", nodeName.c_str());
  NN_TNN_Trainer nn_tnn_trainer(n, 0);
  
  while (ros::ok()) 
  {
    nn_tnn_trainer.run();
  }
  
  ros::shutdown();
  return 0;
}
