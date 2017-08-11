#pragma once
#ifndef _CAMERA_DRIVER_H_
#define _CAMERA_DRIVER_H_

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

using namespace std;
using namespace cv;


class Camera_Driver
{
    private:
      string ver_ = "1.0";
      
    public:
      int device_id;
      float resolution_ratio;
      Mat image;
      int width;
      int height;
      Mat projection;
      Mat cameraMatrix;
      Mat distortion;
      
    private:
      VideoCapture cap_;
      int device_;
      bool flag_camera_read;
      bool camera_read();
      void camera_info();

    public:
      Camera_Driver();
      ~Camera_Driver();
      bool run();
      bool is_open(){return flag_camera_read;};
};

Camera_Driver::Camera_Driver()
{
    device_id = 0;
    resolution_ratio = 1.0;
    
    camera_read();
    
    projection = cv::Mat::zeros(3, 4, CV_64F);
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distortion = cv::Mat::zeros(1, 5, CV_64F);
    
    camera_info();
    
    run();
}

Camera_Driver::~Camera_Driver()
{}

bool Camera_Driver::camera_read()
{
    device_ = device_id;
    cap_.open(device_); 
    flag_camera_read = cap_.isOpened();
    return cap_.isOpened();
}

void Camera_Driver::camera_info()
{
    if(image.empty()) 
	  return;
    width = image.cols;
    height = image.rows;
    cameraMatrix.at<double>(0, 0) = height;//colorParams.fx;
    cameraMatrix.at<double>(1, 1) = width;//colorParams.fy;
//  cameraMatrix.at<double>(2, 2) = 10;//colorParams.fy;
//  cameraMatrix.at<float>(0, 2) = 240;//colorParams.cx;
//  cameraMatrix.at<float>(1, 2) = 320;//colorParams.cy;
    cameraMatrix.copyTo(projection(cv::Rect(0, 0, 3, 3)));
}

bool Camera_Driver::run()
{
    if(device_ != device_id)
    {
      device_ = device_id;
      camera_read();
    }
    // Check if video device can be opened with the given index
    if(!flag_camera_read)
      return false;
    cap_ >> this -> image;
    // Check if grabbed frame is actually full with some content
    if(image.empty()) 
	  return false;
    
    if(resolution_ratio < 1.0)
      resize(image, image, Size(image.cols*resolution_ratio, image.rows*resolution_ratio));
    camera_info();
    cv::waitKey(1);
    //img = image;
    return true;
}



#endif
