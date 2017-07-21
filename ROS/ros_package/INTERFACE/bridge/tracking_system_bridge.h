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

//#include <ros/spinner.h>
#include <image_transport/image_transport.h>
//#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
/*
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
*/

using namespace ros;
using namespace std;
using namespace cv;


class Tracking_System_Bridge
{
  private:
    float ver_;
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    
    /*
    message_filters::Subscriber<std_msgs::String> *image_tracked_region_status_sub_;
    message_filters::Subscriber<std_msgs::String> *image_tracked_region_region_sub_;
    message_filters::Subscriber<std_msgs::Int32> *image_tracked_region_x_sub_;
    message_filters::Subscriber<std_msgs::Int32> *image_tracked_region_y_sub_;
    message_filters::Subscriber<std_msgs::Int32> *image_tracked_region_width_sub_;
    message_filters::Subscriber<std_msgs::Int32> *image_tracked_region_height_sub_;
    typedef message_filters::sync_policies::ExactTime<std_msgs::String, std_msgs::String
                                                     ,std_msgs::Int32, std_msgs::Int32
                                                     ,std_msgs::Int32, std_msgs::Int32> ExactSyncPolicy;
    */
    /*
    ros::AsyncSpinner spinner;
    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    */
    image_transport::Subscriber image_tracked_sub_;
    image_transport::Subscriber image_disp_sub_;
    ros::Publisher  activation_pub;
    ros::Publisher  activation_face_pub;
    ros::Publisher  activation_disparity_pub;
    ros::Subscriber image_tracked_region_status_sub_;
    ros::Subscriber image_tracked_region_region_sub_;
    
    int queue_size;
    cv_bridge::CvImagePtr cv_ptr;
    std_msgs::Bool    msg_activation_;
    std_msgs::Bool    msg_activation_face_;
    std_msgs::Bool    msg_activation_disparity_;
    
    
  private:
    void pub_init();
    void sub_init();
    void image_tracked_callBack(const sensor_msgs::ImageConstPtr& msg);
	void image_disp_callBack(const sensor_msgs::ImageConstPtr& msg);
    void tracked_region_status_callBack(const std_msgs::String::ConstPtr& msg);
    void tracked_region_region_callBack(const std_msgs::String::ConstPtr& msg);
    /*
    void callback(const std_msgs::String::ConstPtr& status, const std_msgs::String::ConstPtr& region,
                  const std_msgs::Int32 region_x, const std_msgs::Int32 region_y,
                  const std_msgs::Int32 region_width, const std_msgs::Int32 region_height);
    */

  public:
    string topic_activation_pub;
    string topic_activation_face_pub;
    string topic_activation_disparity_pub;
    string topic_image_tracked_sub;
    string topic_image_disp_sub;
    string topic_image_tracked_region_status_sub;
    string topic_image_tracked_region_region_sub;
    /*
    string topic_image_tracked_region_x_sub;
    string topic_image_tracked_region_y_sub;
    string topic_image_tracked_region_width_sub;
    string topic_image_tracked_region_height_sub;
    */
    Mat image;
    Mat image_disp;
    string tracking_region_status;
    string tracking_region_region;
    
    bool flag_update_image;
    bool flag_update_status;
    bool flag_update_region;
    
  private:
    void msg_decode();
  public:
    bool activation;
    bool activation_face;
    bool activation_disparity;
    bool tracking_lock;
    int tracking_region_x;
    int tracking_region_y;
    int tracking_region_width;
    int tracking_region_height;
    
  public:
    Tracking_System_Bridge(ros::NodeHandle& n);
    ~Tracking_System_Bridge();
    void run();
};

Tracking_System_Bridge::Tracking_System_Bridge(ros::NodeHandle& n) : n_(n), it_(n_)//, spinner(0)
{
  ver_ = 1.0;
  topic_activation_pub = "tracking_system/activation";
  topic_activation_face_pub = "tracking_system/activation/face";
  topic_activation_disparity_pub = "tracking_system/activation/disparity";
  topic_image_tracked_sub = "tracking_system/image_tracked";
  topic_image_disp_sub = "tracking_system/image_disp";
  topic_image_tracked_region_status_sub = "tracking_system/status";
  topic_image_tracked_region_region_sub = "tracking_system/region";
  /*
  topic_image_tracked_region_x_sub = "tracking_system/region_x";
  topic_image_tracked_region_y_sub = "tracking_system/region_y";
  topic_image_tracked_region_width_sub = "tracking_system/region_width";
  topic_image_tracked_region_height_sub = "tracking_system/region_height";
  */
  queue_size = 4;
  activation = true;
  activation_face = true;
  activation_disparity = true;
  flag_update_image = false;
  flag_update_status = false;
  flag_update_region = false;
  
  pub_init();
  sub_init();
  run();
}

Tracking_System_Bridge::~Tracking_System_Bridge()
{
    /*
    delete syncExact;
    delete image_tracked_region_status_sub_;
    delete image_tracked_region_region_sub_;
    delete image_tracked_region_x_sub_;
    delete image_tracked_region_y_sub_;
    delete image_tracked_region_width_sub_;
    delete image_tracked_region_height_sub_;
    */
}
/*
void Tracking_System_Bridge::callback(const std_msgs::String::ConstPtr& status, const std_msgs::String::ConstPtr& region,
                                      const std_msgs::Int32 region_x, const std_msgs::Int32 region_y,
                                      const std_msgs::Int32 region_width, const std_msgs::Int32 region_height)
*/

void Tracking_System_Bridge::run()
{
  msg_activation_.data = activation;
  activation_pub.publish(msg_activation_);
  
  msg_activation_face_.data = activation_face;
  activation_face_pub.publish(msg_activation_face_);
  
  msg_activation_disparity_.data = activation_disparity;
  activation_disparity_pub.publish(msg_activation_disparity_);
}

void Tracking_System_Bridge::tracked_region_status_callBack(const std_msgs::String::ConstPtr& msg)
{
    if(!activation) return;
    tracking_region_status = string(msg -> data);
    /*
    tracking_region_x = region_x.data;
    tracking_region_y = region_y.data;
    tracking_region_width = region_width.data;
    tracking_region_height = region_height.data;
    */
    flag_update_status = true;
    msg_decode();
}

void Tracking_System_Bridge::tracked_region_region_callBack(const std_msgs::String::ConstPtr& msg)
{
    if(!activation) return;
    tracking_region_region = string(msg -> data);
    /*
    tracking_region_x = region_x.data;
    tracking_region_y = region_y.data;
    tracking_region_width = region_width.data;
    tracking_region_height = region_height.data;
    */
    flag_update_region = true;
    msg_decode();
}

void Tracking_System_Bridge::image_tracked_callBack(const sensor_msgs::ImageConstPtr& msg)
{ 
  if(!activation) return;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_ptr -> image.copyTo(this -> image);
  flag_update_image = true;
  //msg_decode();
}

void Tracking_System_Bridge::image_disp_callBack(const sensor_msgs::ImageConstPtr& msg)
{ 
  if(!activation) return;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_ptr -> image.copyTo(this -> image_disp);
  flag_update_image = true;
  //msg_decode();
}

void Tracking_System_Bridge::pub_init()
{
  activation_pub = n_.advertise< std_msgs::Bool>(topic_activation_pub.c_str(), 1);
  activation_face_pub = n_.advertise< std_msgs::Bool>(topic_activation_face_pub.c_str(), 1);
  activation_disparity_pub = n_.advertise< std_msgs::Bool>(topic_activation_disparity_pub.c_str(), 1);
}

void Tracking_System_Bridge::sub_init()
{  
  /*
  image_tracked_region_status_sub_ = new message_filters::Subscriber<std_msgs::String>(n_, topic_image_tracked_region_status_sub.c_str(), queue_size);
  image_tracked_region_region_sub_ = new message_filters::Subscriber<std_msgs::String>(n_, topic_image_tracked_region_region_sub.c_str(), queue_size);
  image_tracked_region_x_sub_ = new message_filters::Subscriber<std_msgs::Int32>(n_, topic_image_tracked_region_x_sub.c_str(), queue_size);
  image_tracked_region_y_sub_ = new message_filters::Subscriber<std_msgs::Int32>(n_, topic_image_tracked_region_y_sub.c_str(), queue_size);
  image_tracked_region_width_sub_ = new message_filters::Subscriber<std_msgs::Int32>(n_, topic_image_tracked_region_width_sub.c_str(), queue_size);
  image_tracked_region_height_sub_ = new message_filters::Subscriber<std_msgs::Int32>(n_, topic_image_tracked_region_height_sub.c_str(), queue_size);
  
  syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queue_size), *image_tracked_region_status_sub_
                                                                                            , *image_tracked_region_region_sub_ 
                                                                                            , *image_tracked_region_x_sub_
                                                                                            , *image_tracked_region_y_sub_
                                                                                            , *image_tracked_region_width_sub_
                                                                                            , *image_tracked_region_height_sub_);
                                                                                           
  syncExact->registerCallback(boost::bind(&Tracking_System_Bridge::callback, this, _1, _2, _3, _4, _5, _6));
  */
  image_tracked_sub_ = it_.subscribe(topic_image_tracked_sub.c_str(), queue_size, &Tracking_System_Bridge::image_tracked_callBack, this);
  image_disp_sub_ = it_.subscribe(topic_image_disp_sub.c_str(), queue_size, &Tracking_System_Bridge::image_disp_callBack, this);
  image_tracked_region_status_sub_ = n_.subscribe(topic_image_tracked_region_status_sub.c_str(), queue_size, &Tracking_System_Bridge::tracked_region_status_callBack, this);
  image_tracked_region_region_sub_ = n_.subscribe(topic_image_tracked_region_region_sub.c_str(), queue_size, &Tracking_System_Bridge::tracked_region_region_callBack, this);
}

void Tracking_System_Bridge::msg_decode()
{
#define TRACKING_MISS "MISS"
#define TRACKING_LOCK "LOCK"
#define TRACKING_X "X"
#define TRACKING_Y "Y"
#define TRACKING_WIDTH "W"
#define TRACKING_HEIGHT "H"

  stringstream token, transform;
  string str, tmp;
  int value;
  token << tracking_region_status; token >> str; token.clear();
  if(str == TRACKING_LOCK)
    tracking_lock = true; 
  else
    tracking_lock = false; 
  
  token << tracking_region_region; 
  while(token >> str)
  {
    tmp = str.substr(1);
    transform << tmp; transform >> value; transform.clear();
    tmp = str.substr(0, 1);
    if(tmp == TRACKING_X)
    {
      tracking_region_x = value;
    }
    if(tmp == TRACKING_Y)
    {
      tracking_region_y = value;
    }
    if(tmp == TRACKING_WIDTH)
    {
      tracking_region_width = value;
    }
    if(tmp == TRACKING_HEIGHT)
    {
      tracking_region_height = value;
    }
  }
}
