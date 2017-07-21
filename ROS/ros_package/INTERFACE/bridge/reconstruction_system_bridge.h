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

#include <cv_bridge/cv_bridge.h>


#include <image_transport/image_transport.h>


#include <message_filters/subscriber.h>


using namespace ros;
using namespace std;
using namespace cv;


class Reconstruction_System_Bridge
{
  private:
    float ver_;
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    
    
    ros::Publisher  disparity_mono_activation_pub_;
    image_transport::Subscriber disprity_mono_depth_sub_;
    
    
    int queue_size;
    cv_bridge::CvImagePtr cv_ptr;
    std_msgs::Bool    msg_disparity_mono_activation_;
    
    
  private:
    void pub_init();
    void sub_init();
    void disparity_mono_image_callBack(const sensor_msgs::ImageConstPtr& msg);
    

  public:
    string topic_disparity_mono_activation_pub;
    string topic_disparity_mono_depth_sub;
    /*
    string topic_image_tracked_region_x_sub;
    string topic_image_tracked_region_y_sub;
    string topic_image_tracked_region_width_sub;
    string topic_image_tracked_region_height_sub;
    */
    bool disparity_mono_activation;
    Mat disparity_mono_image;    
    bool flag_update_disparity_mono_image;
    
    
  public:
    
  public:
    Reconstruction_System_Bridge(ros::NodeHandle& n);
    ~Reconstruction_System_Bridge();
    void run();
};

Reconstruction_System_Bridge::Reconstruction_System_Bridge(ros::NodeHandle& n) : n_(n), it_(n_)//, spinner(0)
{
  ver_ = 1.0;
  topic_disparity_mono_activation_pub = "reconstruction_system/disparity_mono_activation";
  topic_disparity_mono_depth_sub = "reconstruction_system/disparity_mono_image";
  
  queue_size = 4;
  disparity_mono_activation = true;
  flag_update_disparity_mono_image = false;
  
  pub_init();
  sub_init();
  run();
}

Reconstruction_System_Bridge::~Reconstruction_System_Bridge()
{
}


void Reconstruction_System_Bridge::run()
{
  msg_disparity_mono_activation_.data = disparity_mono_activation;
  disparity_mono_activation_pub_.publish(msg_disparity_mono_activation_);
}

void Reconstruction_System_Bridge::disparity_mono_image_callBack(const sensor_msgs::ImageConstPtr& msg)
{ 
  if(!disparity_mono_activation) return;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_ptr -> image.copyTo(this -> disparity_mono_image);
  flag_update_disparity_mono_image = true;
  //msg_decode();
}

void Reconstruction_System_Bridge::pub_init()
{
  disparity_mono_activation_pub_ = n_.advertise< std_msgs::Bool>(topic_disparity_mono_activation_pub.c_str(), 1);
}

void Reconstruction_System_Bridge::sub_init()
{  
  disprity_mono_depth_sub_ = it_.subscribe(topic_disparity_mono_depth_sub.c_str(), queue_size, &Reconstruction_System_Bridge::disparity_mono_image_callBack, this);
}
