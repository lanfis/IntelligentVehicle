#pragma once
#ifndef _FACE_DETECTOR_BRIDGE_H_
#define _FACE_DETECTOR_BRIDGE_H_

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace ros;
using namespace std;
using namespace cv;


class Face_Detector_Bridge
{

  private:
    float ver_ = 1.0;
    string nodeName;
    ros::NodeHandle n_;
    ros::Publisher   activation_face_pub_;
    ros::Publisher   face_idx_pub_;
    ros::Subscriber  face_size_sub_;
    ros::Subscriber  face_roi_sub_;
    ros::Subscriber  status_face_sub_;
    
    int queue_size;    
    
  private:
    bool flag_activation;
    void activation_face_publish();
    void face_idx_publish();
    void face_size_sub_callBack(const std_msgs::Int32::ConstPtr& msg);
    void face_roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg);
    void status_face_callBack(const std_msgs::String::ConstPtr& msg);

  public:
    string topic_activation_face_pub;
    string topic_status_face_sub;
    string topic_face_idx_pub;
    string topic_face_size_sub;
    string topic_face_roi_sub;
    
    bool activation_face;
    int face_idx;
    int face_size;
    Rect face_roi;
    string status_face;
    
    bool update_activation_face;
    bool update_face_idx;
    bool flag_update_face_size;
    bool flag_update_face_roi;
    bool flag_update_status_face;
    
  public:
    bool activation;
    
  public:
    Face_Detector_Bridge(ros::NodeHandle& n, int thread);
    ~Face_Detector_Bridge();
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void run();
};

Face_Detector_Bridge::Face_Detector_Bridge(ros::NodeHandle& n, int thread) : n_(n)//, it_(n_)//, spinner(thread)
{
  nodeName = "Face_Detector_Bridge";
  topic_activation_face_pub = nodeName + "/face_detector/activation";
  topic_status_face_sub = nodeName + "/face_detector/status";
  topic_face_idx_pub = nodeName + "/face_detector/idx";
  topic_face_size_sub = nodeName + "/face_detector/size";
  topic_face_roi_sub = nodeName + "/face_detector/roi";
  
  
  queue_size = 4;
  activation = true;
  activation_face = true;
  update_activation_face = true;
  update_face_idx = true;
  flag_update_face_size = false;
  flag_update_face_roi = false;
  flag_update_status_face = false;
  face_idx = 0;
    
  pub_init();
  pub_topic_get();
  sub_init();
  sub_topic_get();
}

Face_Detector_Bridge::~Face_Detector_Bridge()
{
}

void Face_Detector_Bridge::run()
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
    
    activation_face_publish();
    face_idx_publish();
}

void Face_Detector_Bridge::face_idx_publish()
{
    if(!update_face_idx) return;
    std_msgs::Int32 msg;
    msg.data = face_idx;
    face_idx_pub_.publish(msg);
    update_face_idx = false;
}

void Face_Detector_Bridge::activation_face_publish()
{
    if(!flag_activation) return;
    std_msgs::Bool msg;
    msg.data = activation_face;
    activation_face_pub_.publish(msg);
    update_activation_face = false;
}

void Face_Detector_Bridge::face_size_sub_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    if(!flag_activation) return;
    face_size = msg -> data;
    flag_update_face_size = true;
}

void Face_Detector_Bridge::face_roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg)
{ 
  if(!flag_activation) return;
  face_roi.width  = msg -> width;
  face_roi.height = msg -> height;
  face_roi.x      = msg -> x_offset - face_roi.width/2;
  face_roi.y      = msg -> y_offset - face_roi.height/2;
  flag_update_face_roi = true;
}

void Face_Detector_Bridge::status_face_callBack(const std_msgs::String::ConstPtr& msg)
{
    if(!flag_activation) return;
    status_face = string(msg -> data);
    flag_update_status_face = true;
}


void Face_Detector_Bridge::pub_topic_get()
{
    topic_activation_face_pub = activation_face_pub_.getTopic();
    topic_face_idx_pub = face_idx_pub_.getTopic();
}

void Face_Detector_Bridge::pub_init()
{
  activation_face_pub_ = n_.advertise< std_msgs::Bool>(topic_activation_face_pub.c_str(), queue_size);
  face_idx_pub_ = n_.advertise< std_msgs::Int32>(topic_face_idx_pub.c_str(), queue_size);
}

void Face_Detector_Bridge::pub_shutdown()
{
    face_idx_pub_.shutdown();
}

void Face_Detector_Bridge::sub_topic_get()
{
    topic_face_roi_sub = face_roi_sub_.getTopic();
    topic_status_face_sub = status_face_sub_.getTopic();
    topic_face_size_sub = face_size_sub_.getTopic();
}

void Face_Detector_Bridge::sub_init()
{
  face_roi_sub_ = n_.subscribe(topic_face_roi_sub.c_str(), queue_size, &Face_Detector_Bridge::face_roi_callBack, this);
  status_face_sub_ = n_.subscribe(topic_status_face_sub.c_str(), queue_size, &Face_Detector_Bridge::status_face_callBack, this);
  face_size_sub_ = n_.subscribe(topic_face_size_sub.c_str(), queue_size, &Face_Detector_Bridge::face_size_sub_callBack, this);
}

void Face_Detector_Bridge::sub_shutdown()
{
  status_face_sub_.shutdown();
  face_size_sub_.shutdown();
}
#endif
