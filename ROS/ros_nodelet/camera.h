#pragma once
#ifndef _CAMERA_H_
#define _CAMERA_H_

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
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

using namespace std;
using namespace cv;


class Camera
{
    private:
      float ver_;
      ros::AsyncSpinner spinner;
      ros::NodeHandle n_;
      image_transport::ImageTransport it_;
      sensor_msgs::CameraInfo camera_info_;
      
      image_transport::Publisher it_pub_;
      ros::Publisher camera_info_pub_;
      ros::Subscriber device_id_sub_;
      ros::Subscriber width_sub_;
      ros::Subscriber height_sub_;
      ros::Subscriber activation_sub_;
      ros::Publisher status_pub_;
      
      bool flag_activation;
      int queue_size;
      
    public:
      string nodeName;
      string topic_image_pub;
      string topic_camera_info_pub;
      string topic_device_id_sub;
      string topic_width_sub;
      string topic_height_sub;
      string topic_activation_sub;
      string topic_status_pub;
      
      int device_id;
      Mat image;
      int width;
      int height;
      
      bool activation;
      
    private:
      VideoCapture cap_;
      int device_;
      void device_id_callBack(const std_msgs::Int32::ConstPtr& msg);
      void width_callBack(const std_msgs::Int32::ConstPtr& msg);
      void height_callBack(const std_msgs::Int32::ConstPtr& msg);
      void activation_callBack(const std_msgs::Bool::ConstPtr& msg);
      void createCameraInfo();
      void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const;

    public:
      Camera(int thread);
      ~Camera();
      void pub_topic_get();
      void pub_init();
      void pub_shutdown();
      void sub_topic_get();
      void sub_init();
      void sub_shutdown();
      void status_publish(string status);
      void run();
};

Camera::Camera(int thread) : it_(n_), spinner(thread)
{
    ver_ = 1.1;
    nodeName = "Camera";
    topic_image_pub = nodeName+"/image";
    topic_camera_info_pub = nodeName+"/camera_info";
    topic_device_id_sub = nodeName+"/device_id";
    topic_width_sub = nodeName+"/width";
    topic_height_sub = nodeName+"/height";
    topic_activation_sub = nodeName+"/activation";
    topic_status_pub = nodeName+"/status";
    
    queue_size = 4;
    device_id = 0;
    width = 640;
    height = 480;
    
    device_ = device_id;
    cap_.open(device_); 
    createCameraInfo();
    
    activation = true;
    flag_activation = true;
    //status = 1;
    pub_init();
    pub_topic_get();
    sub_init();
    sub_topic_get();
    status_publish(nodeName + " initializing ...");
    spinner.start();
    run();
}

Camera::~Camera()
{}

void Camera::run()
{
    if(!activation)
    {
        if(flag_activation)
        {
//          pub_shutdown();
//          sub_shutdown();
          flag_activation = false;
        }
        return;
    }
    if(!flag_activation)
    {
      pub_init();
      sub_init();
      flag_activation = true;
    }
    if(device_ != device_id)
    {
        device_ = device_id;
        ROS_WARN("Device is changed to : %d", device_);
        cap_.open(device_);
        camera_info_.height = height;
        camera_info_.width = width;
    }
//    istringstream video_sourceCmd(camera_idx_);
    //int video_source = camera_idx_;
    // Check if it is indeed a number
    /*
    if(!(video_sourceCmd >> video_source)) 
    {
      ROS_WARN("Reading video source fail");
      return ;
    }
    */
    //cv::VideoCapture cap(video_source);
    // Check if video device can be opened with the given index
    if(!cap_.isOpened())
    {
        ROS_WARN("Opening source %d error !", device_);
        return;
    }
    sensor_msgs::ImagePtr msg;
    cap_ >> image;
    // Check if grabbed frame is actually full with some content
    if(!image.empty()) 
    {
      resize(image, image, Size(width, height));
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      it_pub_.publish(msg);
      cv::waitKey(1);
    }
    createCameraInfo();
    camera_info_pub_.publish(camera_info_);
}

void Camera::createCameraInfo()
{
  cv::Size size;
  size.width = width;
  size.height = height;

  cv::Mat proj = cv::Mat::zeros(3, 4, CV_64F);


  Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F);
  cameraMatrix.at<double>(0, 0) = height;//colorParams.fx;
  cameraMatrix.at<double>(1, 1) = width;//colorParams.fy;
//  cameraMatrix.at<double>(2, 2) = 10;//colorParams.fy;
//  cameraMatrix.at<float>(0, 2) = 240;//colorParams.cx;
//  cameraMatrix.at<float>(1, 2) = 320;//colorParams.cy;
  cameraMatrix.copyTo(proj(cv::Rect(0, 0, 3, 3)));

  Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
  createCameraInfo(size, cameraMatrix, distortion, cv::Mat::eye(3, 3, CV_64F), proj, camera_info_);
}

void Camera::createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const
{
  cameraInfo.height = size.height;
  cameraInfo.width = size.width;

  const double *itC = cameraMatrix.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    cameraInfo.K[i] = *itC;
  }

  const double *itR = rotation.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itR)
  {
    cameraInfo.R[i] = *itR;
  }

  const double *itP = projection.ptr<double>(0, 0);
  for(size_t i = 0; i < 12; ++i, ++itP)
  {
    cameraInfo.P[i] = *itP;
  }

  cameraInfo.distortion_model = "plumb_bob";
  cameraInfo.D.resize(distortion.cols);
  const double *itD = distortion.ptr<double>(0, 0);
  for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
  {
    cameraInfo.D[i] = *itD;
  }
}

void Camera::device_id_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    device_id = msg -> data;
}

void Camera::width_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    width = msg -> data;
}

void Camera::height_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    height = msg -> data;
}
void Camera::activation_callBack(const std_msgs::Bool::ConstPtr& msg)
{
    activation = msg -> data;
    if(activation)
    {
      ROS_WARN("%s activated !", nodeName.c_str());
      status_publish(nodeName + " activated !");
    }
    else
    {
      ROS_WARN("%s inactivated !", nodeName.c_str());
      status_publish(nodeName + " inactivated !");
    }
    this -> run();
}

void Camera::status_publish(string status)
{
    std_msgs::String msg;
    msg.data = status.c_str();
    status_pub_.publish(msg);
}

void Camera::pub_topic_get()
{
    topic_image_pub = it_pub_.getTopic();
    topic_camera_info_pub = camera_info_pub_.getTopic();
    topic_status_pub = status_pub_.getTopic();
}

void Camera::pub_init()
{
    it_pub_ = it_.advertise(topic_image_pub.c_str(), queue_size);
    camera_info_pub_ = n_.advertise<sensor_msgs::CameraInfo>(topic_camera_info_pub.c_str(), queue_size);
    status_pub_ = n_.advertise<std_msgs::String>(topic_status_pub.c_str(), queue_size);
}

void Camera::pub_shutdown()
{
    it_pub_.shutdown();
    camera_info_pub_.shutdown();
}

void Camera::sub_topic_get()
{   
    topic_device_id_sub = device_id_sub_.getTopic();
    topic_width_sub = width_sub_.getTopic();
    topic_height_sub = height_sub_.getTopic();
    topic_activation_sub = activation_sub_.getTopic();
}

void Camera::sub_init()
{
    device_id_sub_ = n_.subscribe(topic_device_id_sub.c_str(), queue_size, &Camera::device_id_callBack, this);
    width_sub_ = n_.subscribe(topic_width_sub.c_str(), queue_size, &Camera::width_callBack, this);
    height_sub_ = n_.subscribe(topic_height_sub.c_str(), queue_size, &Camera::height_callBack, this);
    activation_sub_ = n_.subscribe(topic_activation_sub.c_str(), queue_size, &Camera::activation_callBack, this);
}

void Camera::sub_shutdown()
{
    device_id_sub_.shutdown();
    width_sub_.shutdown();
    height_sub_.shutdown();
}

#endif
