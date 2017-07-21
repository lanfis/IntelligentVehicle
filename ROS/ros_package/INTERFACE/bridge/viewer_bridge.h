#pragma once
#ifndef _VIEWER_BRIDGE_H_
#define _VIEWER_BRIDGE_H_

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

using namespace ros;
using namespace std;
using namespace cv;


class Viewer_Bridge
{
  private:
    float ver_ = 1.0;
    string nodeName;
    //ros::AsyncSpinner spinner;
    ros::NodeHandle n_;
    image_transport::ImageTransport it;
    image_transport::Publisher it_viewer_image_pub;
    cv_bridge::CvImagePtr cv_ptr;
    
    
  private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double elapsed;
    
    int queue_size;
    int keyboard_sensity;
    bool flag_update_image;
    bool flag_window;
    bool flag_activation;
    
  private:
    void image_pub();
    void image_size_adjust();
    void text_put();
    char keyboard_read();
    void frame_rate_show();
    void system_status_put();

  public:
    string topic_viewer_image_pub;
    string window_name = "Viewer_Bridge";

    Mat image;
    Mat image_resized;
    int image_width;
    int image_height;

    string text;
    Point text_position;
    Scalar text_color;
    double text_size;
    string keyboard;
    
    bool update_image;
    bool window_show;
    bool activation;
    
    string system_status;
    Point system_status_position;
    Scalar system_status_color;
    double system_status_size;

  public:
    Viewer_Bridge(ros::NodeHandle& n, int thread);
    ~Viewer_Bridge();
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void run();
};

Viewer_Bridge::Viewer_Bridge(ros::NodeHandle& n, int thread) : n_(n), it(n_)//, spinner(thread)
{
  nodeName = "Viewer_Bridge";
  topic_viewer_image_pub = nodeName + "/image";

  image_width = 640;
  image_height = 480;

  text = "\0";
  text_position = Point(0, 20);
  text_color = Scalar(0, 255, 0);
  text_size = 0.5;
  
  system_status = "\0";
  system_status_position = Point(0, image_height-10);
  system_status_color = Scalar(0, 0, 255);
  system_status_size = 0.5;

  queue_size = 4;
  keyboard_sensity = 50;
  update_image = false;
  window_show = true;
  activation = true;
  flag_activation = true;
  pub_init();
  pub_topic_get();
  //spinner.start();
}

Viewer_Bridge::~Viewer_Bridge()
{
}

void Viewer_Bridge::run()
{
  if(!activation)
  {
      if(flag_activation)
      {
        pub_shutdown();
        flag_activation = false;
      }
      return;
  }
  if(!flag_activation)
  {
    pub_init();
    flag_activation = true;
  }
  if(!flag_window)
  {
    if(window_show)
    {
      cv::namedWindow(window_name.c_str());
      flag_window = true;
    }
  }
  else
  {
    if(!window_show)
    {
      cv::destroyWindow(window_name.c_str());
      flag_window = false;
    }
  }
  if(!update_image) return;
  now = std::chrono::high_resolution_clock::now();
  elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
  
  if(flag_window) this -> keyboard = keyboard_read();
  text_put();
//  system_status_put();
  frame_rate_show();
  image_pub();
  start = std::chrono::high_resolution_clock::now();
}

void Viewer_Bridge::image_pub()
{
  sensor_msgs::ImagePtr msg;
  if(!this -> image.empty()) 
  {
    image_size_adjust();
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this -> image_resized).toImageMsg();
    it_viewer_image_pub.publish(msg);
    flag_update_image = true;
    if(flag_window)  cv::imshow(window_name.c_str(), this -> image_resized);
  }
  update_image = false;
}

void Viewer_Bridge::image_size_adjust()
{
  if(image_width != this -> image.cols || image_height != this -> image.rows)
    resize(this -> image, this -> image_resized, Size(image_width, image_height));
  else
    this -> image.copyTo(this -> image_resized);
}

void Viewer_Bridge::text_put()
{
  putText(this -> image
        , this -> text
        , this -> text_position
        , FONT_HERSHEY_SIMPLEX//FONT_HERSHEY_COMPLEX_SMALL
        , this -> text_size
        , this -> text_color
        , 1     //thickness – Thickness of the lines used to draw a text.
        , CV_AA //lineType – Line type. See the line for details.
        , false);//bottomLeftOrigin – When true, the image data origin is at the bottom-left corner. Otherwise, it is at the top-left corner.
}

char Viewer_Bridge::keyboard_read()
{
  int key = cv::waitKey(keyboard_sensity);
  this -> keyboard[0] = char(key & 0xFF);
  return char(key & 0xFF);
}

void Viewer_Bridge::frame_rate_show()
{
  ostringstream oss;
  oss.str("");
  oss << "Time Delay : " << elapsed << " ms";
  putText(this -> image
        , oss.str()
        , Point(0, image_height-10)
        , FONT_HERSHEY_SIMPLEX//FONT_HERSHEY_COMPLEX_SMALL
        , 0.5
        , Scalar(0, 0, 255)
        , 1     //thickness – Thickness of the lines used to draw a text.
        , CV_AA //lineType – Line type. See the line for details.
        , false);
}

void Viewer_Bridge::system_status_put()
{
  if(system_status.length() == 0)
    return;
  string ss;
  ss = "[System] : " + system_status;
  putText(this -> image
        , ss
        , this -> system_status_position
        , FONT_HERSHEY_SIMPLEX//FONT_HERSHEY_COMPLEX_SMALL
        , this -> system_status_size
        , this -> system_status_color
        , 1     //thickness – Thickness of the lines used to draw a text.
        , CV_AA //lineType – Line type. See the line for details.
        , false);
}

void Viewer_Bridge::pub_topic_get()
{
    topic_viewer_image_pub = it_viewer_image_pub.getTopic();
}

void Viewer_Bridge::pub_init()
{  
  it_viewer_image_pub = it.advertise(topic_viewer_image_pub.c_str(), queue_size);
}

void Viewer_Bridge::pub_shutdown()
{
  it_viewer_image_pub.shutdown();
}

#endif
