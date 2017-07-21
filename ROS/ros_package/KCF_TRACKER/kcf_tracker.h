#pragma once
#ifndef _KCF_TRACKER_H_
#define _KCF_TRACKER_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>
#include <cstring>
#include <sstream>

#include "KCF/KCF.h"



using namespace ros;
using namespace std;



class KCF_Tracker
{
  public:
    string nodeName;
    string topic_image_sub;
    string topic_roi_sub;
    string topic_track_pub;
    string topic_activation_sub;
    string topic_status_pub;

  private:
    float ver_ = 1.1;
    int queue_size;
    bool flag_activation;
    bool flag_image_update;
    bool flag_roi_update;
    bool flag_kcf_init;
    void status_publish(string status);
    
    KCF *kcf;
    void init();
    void detect();
    void publish();
    
  private:
    ros::AsyncSpinner spinner;
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber it_image_sub_;
    ros::Subscriber kcf_tracker_roi_sub_;
    ros::Publisher kcf_tracker_track_pub_;
    ros::Subscriber kcf_tracker_activation_sub_;
    ros::Publisher kcf_tracker_status_pub_;

    cv_bridge::CvImagePtr cv_ptr;
    void roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg);
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void activation_callBack(const std_msgs::Bool::ConstPtr& msg);

  public:   
    Mat image;
    Rect roi;
    Rect track;
/*
    float interp_factor; // linear interpolation factor for adaptation
    float sigma; // gaussian kernel bandwidth
    float lambda; // regularization
    int cell_size; // HOG cell size
    int cell_sizeQ; // cell size^2, to avoid repeated operations
    float padding; // extra area surrounding the target
    float output_sigma_factor; // bandwidth of gaussian target
    int template_size; // template size
    float scale_step; // scale step for multi-scale estimation
    float scale_weight;  // to downweight detection scores of other scales for added stability
*/    
    bool activation;

  public:
    KCF_Tracker(int thread);
    ~KCF_Tracker();
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void run();
};

KCF_Tracker::KCF_Tracker(int thread) : it_(n_), spinner(thread)
{ 
  nodeName = "KCF_Tracker";
  topic_image_sub = nodeName+"/image";
  topic_roi_sub = nodeName+"/roi";
  topic_track_pub = nodeName+"/track";
  topic_activation_sub = nodeName+"/activation";
  topic_status_pub = nodeName+"/status";

  queue_size = 4;
  roi.x = 0;
  roi.y = 0;
  roi.width = 32;
  roi.height = 32;
  
  activation = true;
  flag_activation = true;
  flag_image_update = false;
  flag_roi_update = false;
  flag_kcf_init = false;
  
  pub_init();
  pub_topic_get();
  sub_init();
  sub_topic_get();
  status_publish(nodeName + " initializing ...");
  kcf = new KCF;
  spinner.start();
  run();
}

KCF_Tracker::~KCF_Tracker()
{
    delete kcf;
}

void KCF_Tracker::run()
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
    
    if(flag_roi_update)
    {
        init();
        flag_roi_update = false;
    }
    if(flag_image_update)
    {
        detect();
        publish();
        flag_image_update = false;
    }
}

void KCF_Tracker::init()
{
    if(!image.empty())
    {
      kcf -> init(image, roi);
      flag_kcf_init = true;
    }
}

void KCF_Tracker::detect()
{
  if(flag_kcf_init)
    kcf -> run(image, track);
  else
  {
    init();
  }
}

void KCF_Tracker::publish()
{
    sensor_msgs::RegionOfInterest msg_track;
    msg_track.x_offset = track.x + track.width/2;      
    msg_track.y_offset = track.y + track.height/2;      
    msg_track.width    = track.width;      
    msg_track.height   = track.height;     
    kcf_tracker_track_pub_.publish(msg_track);
}

void KCF_Tracker::roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg)
{
    roi.width  = msg -> width;
    roi.height = msg -> height;
    roi.x      = msg -> x_offset - roi.width/2;
    roi.y      = msg -> y_offset - roi.height/2;
    flag_roi_update = true;
}

void KCF_Tracker::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image = cv_ptr -> image;
    if(!image.empty())
      flag_image_update = true;
}

void KCF_Tracker::activation_callBack(const std_msgs::Bool::ConstPtr& msg)
{
    activation = msg -> data;
    if(activation)
    {
//      ROS_WARN("%s activated !", nodeName.c_str());
      status_publish(nodeName + " activated !");
    }
    else
    {
//      ROS_WARN("%s inactivated !", nodeName.c_str());
      status_publish(nodeName + " inactivated !");
    }
    this -> run();
}

void KCF_Tracker::status_publish(string status)
{
    std_msgs::String msg;
    msg.data = status.c_str();
    kcf_tracker_status_pub_.publish(msg);
}

void KCF_Tracker::pub_topic_get()
{
    topic_track_pub = kcf_tracker_track_pub_.getTopic();
    topic_status_pub = kcf_tracker_status_pub_.getTopic();
}

void KCF_Tracker::pub_init()
{
  kcf_tracker_track_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_track_pub.c_str(), queue_size);
  kcf_tracker_status_pub_ = n_.advertise< std_msgs::String>(topic_status_pub.c_str(), queue_size);
}

void KCF_Tracker::pub_shutdown()
{
  kcf_tracker_track_pub_.shutdown();
}

void KCF_Tracker::sub_topic_get()
{
    topic_image_sub = it_image_sub_.getTopic();
    topic_roi_sub = kcf_tracker_roi_sub_.getTopic();
    topic_activation_sub = kcf_tracker_activation_sub_.getTopic();
}

void KCF_Tracker::sub_init()
{
  it_image_sub_ = it_.subscribe(topic_image_sub.c_str(), queue_size, &KCF_Tracker::image_callBack, this);
  kcf_tracker_roi_sub_ = n_.subscribe(topic_roi_sub.c_str(), queue_size, &KCF_Tracker::roi_callBack, this);
  kcf_tracker_activation_sub_ = n_.subscribe(topic_activation_sub.c_str(), queue_size, &KCF_Tracker::activation_callBack, this);
}

void KCF_Tracker::sub_shutdown()
{
  it_image_sub_.shutdown();
  kcf_tracker_roi_sub_.shutdown();
}
#endif
