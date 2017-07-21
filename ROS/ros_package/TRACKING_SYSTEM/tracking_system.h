#pragma once
#ifndef _TRACKING_SYSTEM_H_
#define _TRACKING_SYSTEM_H_

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

#include "KCF_TRACKER/kcf_tracker_bridge.h"
#include "FACE_DETECTOR/face_detector_bridge.h"



using namespace ros;
using namespace std;



class Tracking_System
{
  public:
    string nodeName;
    string topic_image_pub;
    string topic_track_pub;
    string topic_lock_pub;
    string topic_activation_sub;
    string topic_status_pub;

  private:
    string ver_ = "D-1";
    int queue_size;
    bool flag_activation;
    bool flag_image_update;
    bool flag_track_update;
    
    KCF_Tracker_Bridge *kcf;
    void kcf_tracker_act();
    void kcf_tracker_inact();
    void kcf_tracker_init();
    Face_Detector_Bridge *fd;
    void face_detector_act();
    void face_detector_inact();
    void face_detector_init();
    
  private:
    ros::AsyncSpinner spinner;
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    cv_bridge::CvImagePtr cv_ptr;
    
    image_transport::Publisher tracking_system_image_pub_;
    ros::Publisher tracking_system_track_pub_;
    ros::Publisher tracking_system_lock_pub_;
    ros::Subscriber tracking_system_activation_sub_;
    ros::Publisher tracking_system_status_pub_;
    
    void tracking_system_image_publish();
    void tracking_system_track_publish();
    void tracking_system_lock_publish();
    void activation_callBack(const std_msgs::Bool::ConstPtr& msg);
    void status_publish(string status);

  public:   
    Mat image;
    Rect track;
    
    bool activation;
    bool update_image;
    bool update_track;

  public:
    Tracking_System(int thread);
    ~Tracking_System();
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void run();
    
  private:
    Rect trace_window_;
    Rect tracker_box_;
    double tracker_center_miss_torlence_dist;
    bool flag_tracker_init_;
    bool flag_tracker_lock_;
    
    void core();
    bool core_tracking_miss();
    bool core_tracking_lock();
};

Tracking_System::Tracking_System(int thread) : it_(n_), spinner(thread)
{ 
  nodeName = "Tracking_System";
  topic_image_pub = nodeName+"/image";
  topic_track_pub = nodeName+"/track";
  topic_lock_pub = nodeName+"/lock";
  topic_activation_sub = nodeName+"/activation";
  topic_status_pub = nodeName+"/status";

  queue_size = 4;
  track.x = 0;
  track.y = 0;
  track.width = 32;
  track.height = 32;
  
  activation = true;
  update_image = false;
  update_track = false;
  
  flag_activation = true;
  flag_image_update = false;
  flag_track_update = false;
  
  pub_init();
  pub_topic_get();
  sub_init();
  sub_topic_get();
  status_publish(nodeName + " initializing ...");
  status_publish(nodeName + "/kcf_tracker object initializing ...");
  kcf_tracker_init();
  status_publish(nodeName + "/face_detector object initializing ...");
  face_detector_init();
  
  tracker_center_miss_torlence_dist = 200;
  flag_tracker_init_ = false;
  flag_tracker_lock_ = false;  
  trace_window_ = track;
  tracker_box_ = track;
  
  spinner.start();
  run();
}

Tracking_System::~Tracking_System()
{
    if(kcf != NULL)
      delete kcf;
    if(fd != NULL)
      delete fd;
}

void Tracking_System::run()
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
    
    core();
    kcf -> run();
    fd -> run();
    
//  tracking_system_image_publish();
    tracking_system_track_publish();
    tracking_system_lock_publish();
}

void Tracking_System::core()
{
  if(!flag_tracker_init_)
  {
    cout << "kcf_tracker initializing \n";
    status_publish(nodeName + "/kcf_tracker initializing ...");
    kcf_tracker_act();
    kcf -> kcf_roi = tracker_box_;
    kcf -> update_kcf_roi = true;
    kcf -> run();
    trace_window_ = tracker_box_;
    flag_tracker_init_ = true;
    return;
  }
  
  if(core_tracking_miss())
  {
    track = trace_window_;
    return;
  }
  if(core_tracking_lock())
  {
    track = trace_window_;
    return;
  }
  track = trace_window_;
  return;
}

bool Tracking_System::core_tracking_miss()
{
  if(flag_tracker_lock_) return false;
  //ROS_WARN("%s tracking miss !", nodeName.c_str());
  face_detector_act();
  if(fd -> flag_update_face_roi)
  {
    fd -> face_idx = 0;
    fd -> update_face_idx = true;
      
    tracker_box_ = fd -> face_roi;
    trace_window_ = tracker_box_;
    fd -> flag_update_face_roi = false;
    flag_tracker_lock_ = true;
cout << "asdfasdf\n";
    return true;
  }
  else
  {
    if(!kcf -> flag_update_kcf_track) return false;
    tracker_box_ = kcf -> kcf_track;
/*
    double dist = norm(Mat(Point(trace_window_.x, trace_window_.y)), Mat(Point(tracker_box_.x, tracker_box_.y)));
    if(dist < tracker_center_miss_torlence_dist)
    {
      kcf -> kcf_roi = tracker_box_;
      kcf -> update_kcf_roi = true;
      trace_window_ = tracker_box_;
    }
*/
    kcf -> flag_update_kcf_track = false;
cout << "qwerqw\n";
  }
  return false;
}

bool Tracking_System::core_tracking_lock()
{
  if(!flag_tracker_lock_) return false;
  face_detector_inact();

  if(!kcf -> flag_update_kcf_track) return false;
  tracker_box_ = kcf -> kcf_track;
  kcf -> flag_update_kcf_track = false;
  double dist = norm(Mat(Point(trace_window_.x, trace_window_.y)), Mat(Point(tracker_box_.x, tracker_box_.y)));
  if((dist > tracker_center_miss_torlence_dist))// || (face_detector_.face.size() == 1 && norm(Mat(Point(trace_window_.x, trace_window_.y)), Mat(Point(face_detector_.face[0].x, face_detector_.face[0].y))) > double(tracker_center_miss_torlence_dist))))
  {
    kcf -> kcf_roi = trace_window_;
    kcf -> update_kcf_roi = true;
    flag_tracker_lock_ = false;
cout << "zxczxcz\n";
    return true;
  }
cout << "tttt\n";
  /*
  else
  {
    if(face_detector_.face.size() > 0 && norm(Mat(Point(tracker_box_.x, tracker_box_.y)), Mat(Point(face_detector_.face[0].x, face_detector_.face[0].y))) < tracker_center_miss_torlence_dist)
    {
      tracker_box_ = face_detector_.face[0];
      tracker_kcf_.init( tracker_box_, image );
    }
    trace_window_ = tracker_box_;
  }
  */
  /*
  if((trace_window_.x + trace_window_.width) > image.cols || (trace_window_.x) < 0)
    flag_tracker_lock_ = false;
  if((trace_window_.y + trace_window_.height) > image.rows || (trace_window_.y) < 0)
    flag_tracker_lock_ = false;
  */
  return false;
}

void Tracking_System::kcf_tracker_act()
{
  kcf -> update_activation_kcf = true;
  kcf -> run();
}

void Tracking_System::kcf_tracker_inact()
{
  kcf -> update_activation_kcf = false;
  kcf -> run();
}

void Tracking_System::kcf_tracker_init()
{
  kcf = new KCF_Tracker_Bridge(n_, 0);
  kcf -> activation = true;
  kcf -> update_activation_kcf = true;
  kcf -> update_kcf_roi = false;
  kcf -> run();
}

void Tracking_System::face_detector_act()
{
  fd -> update_activation_face = true;
  fd -> run();
}

void Tracking_System::face_detector_inact()
{
  fd -> update_activation_face = false;
  fd -> run();
}

void Tracking_System::face_detector_init()
{
  fd = new Face_Detector_Bridge(n_, 0);
  fd -> activation = true;   
  fd -> update_activation_face = false;
  fd -> update_face_idx = false;
  fd -> run();
}

void Tracking_System::tracking_system_image_publish()
{
  sensor_msgs::ImagePtr msg;
  if(!this -> image.empty()) 
  {
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this -> image).toImageMsg();
    tracking_system_image_pub_.publish(msg);
    flag_image_update = true;
  }
  update_image = false;
}

void Tracking_System::tracking_system_track_publish()
{
    sensor_msgs::RegionOfInterest msg_track;
    msg_track.x_offset = track.x + track.width/2;      
    msg_track.y_offset = track.y + track.height/2;      
    msg_track.width    = track.width;      
    msg_track.height   = track.height;     
    tracking_system_track_pub_.publish(msg_track);
}

void Tracking_System::tracking_system_lock_publish()
{
    std_msgs::Bool msg;
    msg.data = flag_tracker_lock_;
    tracking_system_lock_pub_.publish(msg);
}

void Tracking_System::activation_callBack(const std_msgs::Bool::ConstPtr& msg)
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

void Tracking_System::status_publish(string status)
{
    std_msgs::String msg;
    msg.data = status.c_str();
    tracking_system_status_pub_.publish(msg);
}
    
void Tracking_System::pub_topic_get()
{
    topic_image_pub = tracking_system_image_pub_.getTopic();
    topic_track_pub = tracking_system_track_pub_.getTopic();
    topic_lock_pub = tracking_system_lock_pub_.getTopic();
    topic_status_pub = tracking_system_status_pub_.getTopic();
}

void Tracking_System::pub_init()
{
  tracking_system_image_pub_ = it_.advertise(topic_image_pub.c_str(), queue_size);
  tracking_system_track_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_track_pub.c_str(), queue_size);
  tracking_system_lock_pub_ = n_.advertise< std_msgs::Bool>(topic_lock_pub.c_str(), queue_size);
  tracking_system_status_pub_ = n_.advertise< std_msgs::String>(topic_status_pub.c_str(), queue_size);
}

void Tracking_System::pub_shutdown()
{
  tracking_system_image_pub_.shutdown();
  tracking_system_track_pub_.shutdown();
  tracking_system_lock_pub_.shutdown();
}

void Tracking_System::sub_topic_get()
{
    topic_activation_sub = tracking_system_activation_sub_.getTopic();
}

void Tracking_System::sub_init()
{
  tracking_system_activation_sub_ = n_.subscribe(topic_activation_sub.c_str(), queue_size, &Tracking_System::activation_callBack, this);
}

void Tracking_System::sub_shutdown()
{
}
#endif
