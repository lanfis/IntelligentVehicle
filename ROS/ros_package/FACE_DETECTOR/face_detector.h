#pragma once
#ifndef _FACE_DETECTOR_H_
#define _FACE_DETECTOR_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <string>
#include <cstring>
#include <sstream>

#include "FACE/Face_Detector_Cascade.h"
//#include "VISION_SYSTEM/camera.h"



using namespace ros;
using namespace std;



class Face_Detector
{
  public:
    string nodeName;
    string topic_image_sub;
    string topic_face_idx_sub;
    string topic_face_size_pub;
    string topic_face_roi_pub;
    string topic_activation_sub;
    string topic_status_pub;

  private:
    float ver_ = 1.1;
    int queue_size;
    bool flag_activation;
    bool flag_image_update;
    
    Face_Detector_Cascade *fdc;
    void detect();
    void publish();
    
  private:
    ros::AsyncSpinner spinner;
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber it_image_sub_;
    ros::Subscriber face_detector_face_idx_sub_;
    ros::Publisher face_detector_face_size_pub_;
    ros::Publisher face_detector_face_roi_pub_;
    ros::Subscriber face_detector_activation_sub_;
    ros::Publisher face_detector_status_pub_;

    cv_bridge::CvImagePtr cv_ptr;
    void status_publish(string status);
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void face_idx_callBack(const std_msgs::Int32::ConstPtr& msg);
    void activation_callBack(const std_msgs::Bool::ConstPtr& msg);

  public:   
    Mat image;
    Rect face_roi;
    int face_idx;
    int face_size;
    
    bool activation;

  public:
    Face_Detector(int thread);
    ~Face_Detector();
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void run();
};

Face_Detector::Face_Detector(int thread) : it_(n_), spinner(thread)
{ 
  nodeName = "Face_Detector";
  topic_image_sub = nodeName+"/image";
  topic_face_idx_sub = nodeName+"/face_idx";
  topic_face_size_pub = nodeName+"/face_size";
  topic_face_roi_pub = nodeName+"/face_roi";
  topic_activation_sub = nodeName+"/activation";
  topic_status_pub = nodeName+"/status";

  queue_size = 4;
  face_idx = 0;
  face_size = 0;
  face_roi.x = 0;
  face_roi.y = 0;
  face_roi.width = 0;
  face_roi.height = 0;
  
  activation = true;
  flag_activation = true;
  flag_image_update = false;
  
  pub_init();
  pub_topic_get();
  sub_init();
  sub_topic_get();
  status_publish(nodeName + " initializing ...");
  fdc = new Face_Detector_Cascade;
  
  spinner.start();
  run();
}

Face_Detector::~Face_Detector()
{
    delete fdc;
}

void Face_Detector::run()
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
    
    if(flag_image_update)
    {
        detect();
        publish();
        flag_image_update = false;
    }
}

void Face_Detector::detect()
{
    fdc -> face_detect(image);
    face_size = fdc -> face.size();
    if(face_idx < face_size)
    {
      face_roi = fdc -> face[face_idx];
    }
}

void Face_Detector::publish()
{
    std_msgs::Int32 msg;
    msg.data = face_size;   face_detector_face_size_pub_.publish(msg);
    if(face_size > 0)
    {
      sensor_msgs::RegionOfInterest msg_roi;
      msg_roi.x_offset = face_roi.x + face_roi.width/2;      
      msg_roi.y_offset = face_roi.y + face_roi.height/2;      
      msg_roi.width    = face_roi.width;      
      msg_roi.height   = face_roi.height;      
      face_detector_face_roi_pub_.publish(msg_roi);
    }
}

void Face_Detector::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    if(!flag_activation) return;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image = cv_ptr -> image;
    if(!image.empty())
      flag_image_update = true;
}

void Face_Detector::face_idx_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    if(!flag_activation) return;
    if(msg -> data > face_size)
    {
        status_publish(nodeName + " Face idx Error !");
    }
    else
        face_idx = msg -> data;
}

void Face_Detector::activation_callBack(const std_msgs::Bool::ConstPtr& msg)
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

void Face_Detector::status_publish(string status)
{
    std_msgs::String msg;
    msg.data = status.c_str();
    face_detector_status_pub_.publish(msg);
}

void Face_Detector::pub_topic_get()
{
    topic_face_size_pub = face_detector_face_size_pub_.getTopic();
    topic_face_roi_pub = face_detector_face_roi_pub_.getTopic();
    topic_status_pub = face_detector_status_pub_.getTopic();
}

void Face_Detector::pub_init()
{
  face_detector_face_size_pub_ = n_.advertise< std_msgs::Int32>(topic_face_size_pub.c_str(), queue_size);
  face_detector_face_roi_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_face_roi_pub.c_str(), queue_size);
  face_detector_status_pub_ = n_.advertise< std_msgs::String>(topic_status_pub.c_str(), queue_size);
}

void Face_Detector::pub_shutdown()
{
  face_detector_face_size_pub_.shutdown();
  face_detector_face_roi_pub_.shutdown();
}

void Face_Detector::sub_topic_get()
{
    topic_image_sub = it_image_sub_.getTopic();
    topic_face_idx_sub = face_detector_face_idx_sub_.getTopic();
    topic_activation_sub = face_detector_activation_sub_.getTopic();
}

void Face_Detector::sub_init()
{
  it_image_sub_ = it_.subscribe(topic_image_sub.c_str(), queue_size, &Face_Detector::image_callBack, this);
  face_detector_face_idx_sub_ = n_.subscribe(topic_face_idx_sub.c_str(), queue_size, &Face_Detector::face_idx_callBack, this);
  face_detector_activation_sub_ = n_.subscribe(topic_activation_sub.c_str(), queue_size, &Face_Detector::activation_callBack, this);
}

void Face_Detector::sub_shutdown()
{
  it_image_sub_.shutdown();
  face_detector_face_idx_sub_.shutdown();
}
#endif
