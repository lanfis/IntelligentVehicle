#pragma once
#ifndef _MATRIX_NODE_H_
#define _MATRIX_NODE_H_

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

#include "MATRIX/Matrix.h"

using namespace std;
using namespace cv;


class Matrix_Node
{
    private:
      string ver_ = "1.1";
      ros::AsyncSpinner spinner;
      ros::NodeHandle n_;
	  Matrix *mat_node;
	  
    public:
      Matrix_Node(ros::NodeHandle& n, int thread);
      ~Matrix_Node();
      void run();
};

Matrix_Node::Matrix_Node(ros::NodeHandle& n, int thread) : n_(n), spinner(thread)
{
	mat_node = new Matrix(n_);
	mat_node -> init();
    run();
    spinner.start();
}

Matrix_Node::~Matrix_Node()
{
	delete mat_node;
}

void Matrix_Node::run()
{
	mat_node -> run();
}


#endif
