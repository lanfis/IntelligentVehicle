#pragma once
#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "../../ros_object/PLATFORM_CONTROL/platform_control.h"

using namespace std;
using namespace cv;


class Platform
{
    private:
      string ver_ = "1.1";
      ros::AsyncSpinner spinner;
      ros::NodeHandle n_;
	  Platform_Control *pc;
	  
    public:
      Platform(ros::NodeHandle& n, int thread);
      ~Platform();
      void run();
};

Platform::Platform(ros::NodeHandle& n, int thread) : n_(n), spinner(thread)
{
	pc = new Platform_Control(n_);
	pc -> init();
    run();
    spinner.start();
}

Platform::~Platform()
{
	delete pc;
}

void Platform::run()
{
	pc -> run();
}


#endif
