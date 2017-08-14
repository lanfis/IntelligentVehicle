#pragma once
#ifndef _ROS_NN_SYSTEM_H_
#define _ROS_NN_SYSTEM_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "TNN/TNN_Trainer.h"

using namespace std;
using namespace cv;


class NN_TNN_Trainer
{
    private:
      string ver_ = "1.1";
      ros::AsyncSpinner spinner;
      ros::NodeHandle n_;
	  TNN_Trainer *tnn_trainer;
	  
    public:
      NN_TNN_Trainer(ros::NodeHandle& n, int thread);
      ~NN_TNN_Trainer();
      void run();
};

NN_TNN_Trainer::NN_TNN_Trainer(ros::NodeHandle& n, int thread) : n_(n), spinner(thread)
{
    spinner.start();
	tnn_trainer = new TNN_Trainer(n_);
	tnn_trainer -> init();
	tnn_trainer -> train_init();
	tnn_trainer -> mnist_load();
    run();
}

NN_TNN_Trainer::~NN_TNN_Trainer()
{
	delete tnn_trainer;
}

void NN_TNN_Trainer::run()
{
	tnn_trainer -> run();
}


#endif
