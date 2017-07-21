#pragma once
#ifndef _KCF_TRACKER_BRIDGE_H_
#define _KCF_TRACKER_BRIDGE_H_

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
#include <std_msgs/Bool.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace ros;
using namespace std;
using namespace cv;


class KCF_Tracker_Bridge
{

  private:
    float ver_ = 1.0;
    string nodeName;
    ros::NodeHandle n_;
    ros::Publisher   activation_kcf_pub_;
    ros::Publisher   kcf_roi_pub_;
    ros::Subscriber  kcf_track_sub_;
    ros::Subscriber  status_kcf_sub_;
    
    int queue_size;    
    
  private:
    bool flag_activation;
    void activation_kcf_publish();
    void kcf_roi_publish();
    void kcf_track_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg);
    void status_kcf_callBack(const std_msgs::String::ConstPtr& msg);

  public:
    string topic_activation_kcf_pub;
    string topic_kcf_roi_pub;
    string topic_kcf_track_sub;
    string topic_status_kcf_sub;
    
    bool activation_kcf;
    Rect kcf_roi;
    Rect kcf_track;
    string status_kcf;
    
    bool update_activation_kcf;
    bool update_kcf_roi;
    bool flag_update_kcf_track;
    bool flag_update_status_kcf;
    
  public:
    bool activation;
    
  public:
    KCF_Tracker_Bridge(ros::NodeHandle& n, int thread);
    ~KCF_Tracker_Bridge();
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void run();
};

KCF_Tracker_Bridge::KCF_Tracker_Bridge(ros::NodeHandle& n, int thread) : n_(n)//, it_(n_)//, spinner(thread)
{
  nodeName = "KCF_Tracker_Bridge";
  topic_activation_kcf_pub = nodeName + "/kcf_tracker/activation";
  topic_status_kcf_sub = nodeName + "/kcf_tracker/status";
  topic_kcf_roi_pub = nodeName + "/kcf_tracker/roi";
  topic_kcf_track_sub = nodeName + "/kcf_tracker/track";
  
  
  queue_size = 4;
  activation = true;
  activation_kcf = true;
  update_activation_kcf = true;
  update_kcf_roi = false;
  flag_update_kcf_track = false;
  flag_update_status_kcf = false;
  
  
  pub_init();
  pub_topic_get();
  sub_init();
  sub_topic_get();
}

KCF_Tracker_Bridge::~KCF_Tracker_Bridge()
{
}

void KCF_Tracker_Bridge::run()
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
    
    activation_kcf_publish();
    kcf_roi_publish();
}

void KCF_Tracker_Bridge::kcf_roi_publish()
{
    if(!update_kcf_roi) return;
    sensor_msgs::RegionOfInterest msg;
    msg.x_offset = kcf_roi.x + kcf_roi.width/2;      
    msg.y_offset = kcf_roi.y + kcf_roi.height/2;      
    msg.width    = kcf_roi.width;      
    msg.height   = kcf_roi.height;      
    kcf_roi_pub_.publish(msg);
    update_kcf_roi = false;
}

void KCF_Tracker_Bridge::activation_kcf_publish()
{
    if(!flag_activation) return;
    std_msgs::Bool msg;
    msg.data = activation_kcf;
    activation_kcf_pub_.publish(msg);
    update_activation_kcf = false;
}

void KCF_Tracker_Bridge::kcf_track_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg)
{ 
  if(!flag_activation) return;
  kcf_track.width  = msg -> width;
  kcf_track.height = msg -> height;
  kcf_track.x      = msg -> x_offset - kcf_track.width/2;
  kcf_track.y      = msg -> y_offset - kcf_track.height/2;
  flag_update_kcf_track = true;
}

void KCF_Tracker_Bridge::status_kcf_callBack(const std_msgs::String::ConstPtr& msg)
{
    if(!flag_activation) return;
    status_kcf = string(msg -> data);
    flag_update_status_kcf = true;
}


void KCF_Tracker_Bridge::pub_topic_get()
{
    topic_activation_kcf_pub = activation_kcf_pub_.getTopic();
    topic_kcf_roi_pub = kcf_roi_pub_.getTopic();
}

void KCF_Tracker_Bridge::pub_init()
{
  activation_kcf_pub_ = n_.advertise< std_msgs::Bool>(topic_activation_kcf_pub.c_str(), queue_size);
  kcf_roi_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_kcf_roi_pub.c_str(), queue_size);
}

void KCF_Tracker_Bridge::pub_shutdown()
{
    kcf_roi_pub_.shutdown();
}

void KCF_Tracker_Bridge::sub_topic_get()
{
    topic_kcf_track_sub = kcf_track_sub_.getTopic();
    topic_status_kcf_sub = status_kcf_sub_.getTopic();
}

void KCF_Tracker_Bridge::sub_init()
{
  kcf_track_sub_ = n_.subscribe(topic_kcf_track_sub.c_str(), queue_size, &KCF_Tracker_Bridge::kcf_track_callBack, this);
  status_kcf_sub_ = n_.subscribe(topic_status_kcf_sub.c_str(), queue_size, &KCF_Tracker_Bridge::status_kcf_callBack, this);
}

void KCF_Tracker_Bridge::sub_shutdown()
{
  kcf_track_sub_.shutdown();
}
#endif
