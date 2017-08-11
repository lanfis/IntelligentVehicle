#pragma once
#ifndef _VISION_SYSTEM_H_
#define _VISION_SYSTEM_H_

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

#include "CAMERA/Camera.h"

using namespace std;
using namespace cv;


class Vision_System
{
    private:
      string ver_ = "1.1";
      ros::AsyncSpinner spinner;
      ros::NodeHandle n_;
	  Camera *camera;
	  
    public:
      Vision_System(ros::NodeHandle& n, int thread);
      ~Vision_System();
      void run();
};

Vision_System::Vision_System(ros::NodeHandle& n, int thread) : n_(n), spinner(thread)
{
    spinner.start();
	camera = new Camera(n_);
	camera -> init();
    run();
}

Vision_System::~Vision_System()
{
	delete camera;
}

void Vision_System::run()
{
	camera -> run();
}


#endif
