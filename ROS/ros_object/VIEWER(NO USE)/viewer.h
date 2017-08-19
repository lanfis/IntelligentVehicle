#pragma once
#ifndef _VIEWER_H_
#define _VIEWER_H_

#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>
#include <cstring>
#include <sstream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/RegionOfInterest.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "../../ros_comm/config/console_format.h"


using namespace ros;
using namespace std;
using namespace cv;



class Viewer
{
  public:
    string nodeName = "Viewer";
    string topic_image_sub = "Viewer/image_sub";
    string topic_image_pub = "Viewer/image";
    string topic_activation_sub = "Viewer/activation";
    string topic_status_pub = "Viewer/status";
    string topic_roi_sub = "Viewer/roi_sub";
    
    string window_name = "IV Viewer";

  private:
    string ver_ = "1.0";
    int queue_size = 4;
    bool flag_activation = true;
    bool flag_image_update = false;
    bool flag_window = false;
    bool flag_roi_box_update = false;
    
    string fps_text;
    Point fps_position;
    Scalar fps_color = Scalar(255, 0, 0);
    double fps_size = 0.5;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double elapsed;
    
    int keyboard_sensity = 50;
    
  private:
    char keyboard_read();
    void frame_rate_show();
    void image_size_adjust();
    void roi_show();
    
  private:
    ros::NodeHandle n_;
    image_transport::ImageTransport it_sub_;
    image_transport::ImageTransport it_pub_;
    cv_bridge::CvImagePtr cv_ptr;

    bool useExact = true;
    image_transport::TransportHints hints = "raw";
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::RegionOfInterest> ExactSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::RegionOfInterest> ApproximateSyncPolicy;
    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;
    
    image_transport::SubscriberFilter viewer_image_sub_;
    message_filters::SubscriberFilter<sensor_msgs::RegionOfInterest> viewer_roi_sub_;
    
    //image_transport::Subscriber viewer_image_sub_;
    image_transport::Publisher viewer_image_pub_;
    ros::Subscriber viewer_activation_sub_;
    ros::Publisher viewer_status_pub_;
    //ros::Subscriber viewer_roi_sub_;
    
    void viewer_image_publish();
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void activation_callBack(const std_msgs::Bool::ConstPtr& msg);
    void window_activation_callBack(const std_msgs::Bool::ConstPtr& msg);
    void status_publish(string status);
    void roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg);

  public:   
    Mat image_raw;
    Mat image;
    int image_width = 640;
    int image_height = 480;
    vector<Rect> roi_box;

    string text;
    Point text_position;
    Scalar text_color;
    double text_size;
    
    string keyboard;
    
    bool activation = true;
    bool window_activation = true;
    bool fps_activation = true;

  public:
    Viewer(int thread);
    ~Viewer();
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void run();
    
};

Viewer::Viewer(int thread) : it_sub_(n_), it_pub_(n_)
{ 
  cout << "-- Viewer using resolution in " << image_width << " x " << image_height << " !\n";
  fps_position = Point(0, image_height);
  
  pub_init();
  pub_topic_get();
  sub_init();
  sub_topic_get();
  status_publish(nodeName + " initializing ...");
  
  start = std::chrono::high_resolution_clock::now();
  spinner.start();
  run();
}

Viewer::~Viewer()
{
  if(useExact)
  {
	  delete syncExact;
  }
  else
  {
	  delete syncApproximate;
  }
}

void Viewer::run()
{
    if(!activation)
    {
        if(flag_activation)
        {
//          pub_shutdown();
          flag_activation = false;
        }
        return;
    }
    if(!flag_activation)
    {
//      pub_init();
      flag_activation = true;
    }
    if(!flag_window)
    {
      if(window_activation)
      {
        cv::namedWindow(window_name.c_str());
        flag_window = true;
      }
    }
    else
    {
      if(!window_activation)
      {
        cv::destroyWindow(window_name.c_str());
        flag_window = false;
      }
    }
    
    if(flag_window)
      this -> keyboard = keyboard_read();
    
    if(!flag_image_update) return;
    now = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
    start = now;
	roi_show();
	flag_roi_box_update = false;
    image_size_adjust();
    frame_rate_show();
    viewer_image_publish();
    flag_image_update = false;
}

void Viewer::roi_show()
{
	if(!flag_roi_box_update) return;
	if(image_raw.empty()) return;
	for(int i = 0; i < roi_box.size(); i++)
	{
       rectangle( image_raw, roi_box[i], cv::Scalar(0, 255, 0), 1, 8, 0 );
	}
	roi_box.clear();
}

void Viewer::image_size_adjust()
{
  if(image_width != this -> image_raw.cols || image_height != this -> image_raw.rows)
    resize(this -> image_raw, this -> image, Size(image_width, image_height));
  else
    this -> image = this -> image_raw;//this -> image_raw.copyTo(this -> image);
}

void Viewer::frame_rate_show()
{
  ostringstream oss;
  oss.str("");
  oss << "Delay : " << int(1000000/elapsed) << " fps" << "/ " << int(elapsed/1000) << " ms";
  fps_position = Point(0, image_height-10);
  fps_text = oss.str();
  putText(this -> image
        , fps_text
        , fps_position
        , FONT_HERSHEY_SIMPLEX//FONT_HERSHEY_COMPLEX_SMALL
        , fps_size
        , fps_color
        , 1     //thickness – Thickness of the lines used to draw a text.
        , CV_AA //lineType – Line type. See the line for details.
        , false);
}

char Viewer::keyboard_read()
{
  int key = cv::waitKey(keyboard_sensity);
  this -> keyboard[0] = char(key & 0xFF);
  return char(key & 0xFF);
}

void Viewer::viewer_image_publish()
{
  sensor_msgs::ImagePtr msg;
  if(this -> image.empty()) 
    return;
  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this -> image).toImageMsg();
  viewer_image_pub_.publish(msg);
  if(flag_window)
    imshow(window_name.c_str(), cv_bridge::toCvShare(msg, "bgr8")->image);
}

void Viewer::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    if(!flag_activation)
    {
      flag_image_update = false;
      return;
    }
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//    cv_ptr -> image.copyTo(this -> image_raw);
    this -> image_raw = cv_ptr -> image;
    flag_image_update = true;
}

void Viewer::activation_callBack(const std_msgs::Bool::ConstPtr& msg)
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

void Viewer::window_activation_callBack(const std_msgs::Bool::ConstPtr& msg)
{
    window_activation = msg -> data;
    if(window_activation)
    {
      ROS_WARN("%s activated !", window_name.c_str());
      status_publish(window_name + " activated !");
    }
    else
    {
      ROS_WARN("%s inactivated !", window_name.c_str());
      status_publish(window_name + " inactivated !");
    }
}

void Viewer::status_publish(string status)
{
    std_msgs::String msg;
    msg.data = status.c_str();
    viewer_status_pub_.publish(msg);
}
    
void Viewer::roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg)
{
	Rect roi;
    roi.width  = msg -> width;
    roi.height = msg -> height;
    roi.x      = msg -> x_offset - roi.width/2;
    roi.y      = msg -> y_offset - roi.height/2;
    int xx = roi.x + roi.width;
    int yy = roi.y + roi.height;
    roi.x      = (roi.x < 0)? 0 : roi.x;
    roi.y      = (roi.y < 0)? 0 : roi.y;
    roi.width  = (xx > image_raw.cols)? image_raw.cols - roi.x : xx - roi.x;
    roi.height = (yy > image_raw.rows)? image_raw.rows - roi.y : yy - roi.y;
    this -> roi_box.push_back(roi);
    flag_roi_box_update = true;
}


void Viewer::pub_topic_get()
{
    topic_image_pub = viewer_image_pub_.getTopic();
    topic_status_pub = viewer_status_pub_.getTopic();
}

void Viewer::pub_init()
{
  viewer_image_pub_ = it_pub_.advertise(topic_image_pub.c_str(), queue_size);
  viewer_status_pub_ = n_.advertise< std_msgs::String>(topic_status_pub.c_str(), queue_size);

}

void Viewer::pub_shutdown()
{
  viewer_image_pub_.shutdown();
}

void Viewer::sub_topic_get()
{
    topic_image_sub = viewer_image_sub_.getTopic();
    topic_activation_sub = viewer_activation_sub_.getTopic();
}

void Viewer::sub_init()
{
  viewer_image_sub_ = image_transport::SubscriberFilter(it_sub_, topic_image_sub, queueSize, hints);
//  viewer_image_sub_ = it_sub_.subscribe(topic_image_sub.c_str(), queue_size, &Viewer::image_callBack, this);
  viewer_activation_sub_ = n_.subscribe(topic_activation_sub.c_str(), queue_size, &Viewer::activation_callBack, this);
  viewer_roi_sub_ = n_.subscribe(topic_roi_sub.c_str(), queue_size, &Viewer::roi_callBack, this);
  if(useExact)
  {
    syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), viewer_image_sub_, viewer_roi_sub_);
    syncExact->registerCallback(boost::bind(&Viewer::image_callBack, this, _1, _2));
  }
  else
  {
    syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), viewer_image_sub_, viewer_roi_sub_);
    syncApproximate->registerCallback(boost::bind(&Viewer::image_callBack, this, _1, _2));
  }
}

void Viewer::sub_shutdown()
{
  viewer_image_sub_.shutdown();
  viewer_roi_sub_.shutdown();
}
#endif
