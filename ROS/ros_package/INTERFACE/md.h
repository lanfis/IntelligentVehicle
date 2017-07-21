#pragma once
#ifndef _INTERFACE_H_
#define _INTERFACE_H_

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

#include "bridge/viewer_bridge.h"
#include "FACE_DETECTOR/face_detector_bridge.h"
#include "KCF_TRACKER/kcf_tracker_bridge.h"



using namespace ros;
using namespace std;
using namespace cv;



class Interface
{
  public:
    string nodeName;
/*
    string topic_image_sub;
    string topic_roi_sub;
    string topic_activation_sub;
    string topic_status_pub;
*/

  private:
    float ver_ = 1.0;
    int queue_size;
    bool flag_activation;
    bool flag_image_update;
    bool flag_roi_update;
    
    Viewer_Bridge *vb;
    
  private:
    ros::AsyncSpinner spinner;
    ros::NodeHandle n_;
/*
    image_transport::ImageTransport it_;
    image_transport::Subscriber it_image_sub_;
    ros::Subscriber roi_sub_;
    ros::Subscriber activation_sub_;
*/
    ros::Publisher status_pub_;

//    cv_bridge::CvImagePtr cv_ptr;
    void status_publish(string status);
/*
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg);
    void activation_callBack(const std_msgs::Bool::ConstPtr& msg);
*/
    void view_make();
	
  public:   
    Mat image;
    Rect roi;    
    Scalar roi_color;
    int roi_box_length;
    bool activation;

  public:
    Interface(int thread);
    ~Interface();
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void run();
};

Interface::Interface(int thread) : it_(n_), spinner(thread)
{ 
  nodeName = "Interface";
  topic_image_sub = nodeName+"/image";
  topic_roi_sub = nodeName+"/roi";
  topic_activation_sub = nodeName+"/activation";
  topic_status_pub = nodeName+"/status";

  queue_size = 4;
  roi.x = 0;
  roi.y = 0;
  roi.width = 32;
  roi.height = 32;
  roi_color = Scalar(128, 0, 0);
  roi_box_length = 1;
  
  activation = true;
  flag_activation = true;
  flag_image_update = false;
  flag_roi_update = false;
  
  pub_init();
  pub_topic_get();
  sub_init();
  sub_topic_get();
  vb = new Viewer_Bridge(n_, thread);
  spinner.start();
  run();
}

Interface::~Interface()
{
    delete vb;
}

void Interface::run()
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
        flag_roi_update = false;
    }
    if(flag_image_update)
    {
        view_make();
        vb -> image = this -> image;
        vb -> update_image = true;
        vb -> run();
        flag_image_update = false;
    }
}

void Interface::view_make()
{
    //Point center(roi.x+roi.width/2, roi.y+roi.height/2);
    rectangle(image, roi, roi_color, roi_box_length);
    /*
    if(flag_text_center_position)
    {
      circle(image, center, 1, color, 1, 4, 0);
      string text;
      string tmp;
      stringstream token;
      token << int(center.x);
      token >> tmp;
      text =  "(" + tmp;
      token.clear();
      token << int(center.y);
      token >> tmp;
      text += "," + tmp + ")";
      putText(image
            , text
            , center
            , FONT_HERSHEY_COMPLEX_SMALL
            , 1
            , color);
    }
    */
}

void Interface::roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg)
{
    if(!flag_activation) return;
    roi.x      = msg -> x_offset;
    roi.y      = msg -> y_offset;
    roi.width  = msg -> width;
    roi.height = msg -> height;
    flag_roi_update = true;
}

void Interface::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    if(!flag_activation) return;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image = cv_ptr -> image;
    if(!image.empty())
      flag_image_update = true;
}

void Interface::activation_callBack(const std_msgs::Bool::ConstPtr& msg)
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

void Interface::status_publish(string status)
{
    std_msgs::String msg;
    msg.data = status.c_str();
    status_pub_.publish(msg);
}

void Interface::pub_topic_get()
{
    topic_status_pub = status_pub_.getTopic();
}

void Interface::pub_init()
{
  status_pub_ = n_.advertise< std_msgs::String>(topic_status_pub.c_str(), queue_size);
}

void Interface::pub_shutdown()
{
//  kcf_tracker_track_pub_.shutdown();
}

void Interface::sub_topic_get()
{
    topic_image_sub = it_image_sub_.getTopic();
    topic_roi_sub = roi_sub_.getTopic();
    topic_activation_sub = activation_sub_.getTopic();
}

void Interface::sub_init()
{
  it_image_sub_ = it_.subscribe(topic_image_sub.c_str(), queue_size, &Interface::image_callBack, this);
  roi_sub_ = n_.subscribe(topic_roi_sub.c_str(), queue_size, &Interface::roi_callBack, this);
  activation_sub_ = n_.subscribe(topic_activation_sub.c_str(), queue_size, &Interface::activation_callBack, this);
}

void Interface::sub_shutdown()
{
  it_image_sub_.shutdown();
  roi_sub_.shutdown();
}
#endif
