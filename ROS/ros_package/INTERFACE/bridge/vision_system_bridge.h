#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
using namespace ros;
using namespace std;
using namespace cv;


class Vision_System_Bridge
{
  private:
    float ver_;
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_color_sub_;
    image_transport::Subscriber image_depth_sub_;
    ros::Subscriber camera_info_sub_;
	ros::Publisher device_id_pub_;
	ros::Publisher width_pub_;
	ros::Publisher height_pub_;
	ros::Publisher activation_pub_;
    
    int queue_size;
    cv_bridge::CvImagePtr cv_ptr_color;
    cv_bridge::CvImagePtr cv_ptr_depth;
    sensor_msgs::CameraInfo camera_info_;
    
  private:
    void pub_init();
    void sub_init();
    void image_color_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_depth_callBack(const sensor_msgs::ImageConstPtr& msg);
    void camera_info_callBack(const sensor_msgs::CameraInfo/*::ConstPtr*/ cameraInfo);//, cv::Mat &cameraMatrix) const

  public:
    string topic_image_color_sub;
    string topic_image_depth_sub;
    string topic_camera_info_sub;
    string topic_device_id_pub;
    string topic_width_pub;
    string topic_height_pub;
    string topic_activation_pub;
	
    bool flag_update_image;
    Mat image;
    bool flag_update_depth;
    Mat depth;
    bool flag_update_camera_info;
	int device_id;
    int width;
    int height;
    bool activation;
    
  public:
    Vision_System_Bridge(ros::NodeHandle& n);
    ~Vision_System_Bridge();
    void run();
};

Vision_System_Bridge::Vision_System_Bridge(ros::NodeHandle& n) : n_(n), it_(n_)
{
  ver_ = 1.0;
  topic_image_color_sub = "/image_color";
  topic_image_depth_sub = "/image_depth";
  topic_camera_info_sub = "/camera_info";
  topic_device_id_pub = "/device_id";
  topic_width_pub = "/width";
  topic_height_pub = "/height";
  topic_activation_pub = "/activation";
  
  queue_size = 4;
  device_id = 0;
  width = 640;
  height = 480;
  activation = true;
  flag_update_image = false;
  flag_update_depth = false;
  flag_update_camera_info = false;
  sub_init();
  run();
}

void Vision_System_Bridge::pub_init()
{
    device_id_pub_ = n_.advertise<std_msgs::Int32>(topic_device_id_pub.c_str(), queue_size);
    width_pub_ = n_.advertise<std_msgs::Int32>(topic_width_pub.c_str(), queue_size);
    height_pub_ = n_.advertise<std_msgs::Int32>(topic_height_pub.c_str(), queue_size);
    activation_pub_ = n_.advertise<std_msgs::Bool>(topic_activation_pub.c_str(), queue_size);
}

void Vision_System_Bridge::sub_init()
{
  image_color_sub_ = it_.subscribe(topic_image_color_sub.c_str(), queue_size, &Vision_System_Bridge::image_color_callBack, this);
  image_depth_sub_ = it_.subscribe(topic_image_depth_sub.c_str(), queue_size, &Vision_System_Bridge::image_depth_callBack, this);
  camera_info_sub_ = n_.subscribe(topic_camera_info_sub.c_str(), queue_size, &Vision_System_Bridge::camera_info_callBack, this);
}

Vision_System_Bridge::~Vision_System_Bridge()
{
}

void Vision_System_Bridge::run()
{
	std_msgs::Int32 msg;
	msg.data = device_id;
	device_id_pub_.publish(msg);
	msg.data = width;
	width_pub_.publish(msg);
	msg.data = height;
	height_pub_.publish(msg);
	std_msgs::Bool msg_act;
	msg_act.data = activation;
	activation_pub_.publish(msg_act);
}
  
void Vision_System_Bridge::image_color_callBack(const sensor_msgs::ImageConstPtr& msg)
{ 
  if(!activation) return;
  cv_ptr_color = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_ptr_color -> image.copyTo(this -> image);
  flag_update_image = true;
  /*
  if(flag_img_equalized)
  {
    vector<Mat> channels;//img emphasized
    split(img, channels);
    equalizeHist(channels.at(0), channels.at(0));
    equalizeHist(channels.at(1), channels.at(1));
    equalizeHist(channels.at(2), channels.at(2));
    merge(channels, img);
  }
  
  cvtColor(img, img_gray, CV_BGR2GRAY);
  
  tracker_system(img);        
  img_draw(img, tracker_trace_window_, tracker_window_color_);
  
  image_proc_pub(img);
  msg_transform();
  msg_pub();
  */
}

void Vision_System_Bridge::image_depth_callBack(const sensor_msgs::ImageConstPtr& msg)
{ 
  if(!activation) return;
  cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
  cv_ptr_depth -> image.copyTo(this -> depth);
  flag_update_depth = true;
}
  
void Vision_System_Bridge::camera_info_callBack(const sensor_msgs::CameraInfo/*::ConstPtr*/ cameraInfo)//, cv::Mat &cameraMatrix) const
{
  if(!activation) return;
  /*
  double *itC = cameraMatrix.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    *itC = cameraInfo->K[i];
  }
  */
  camera_info_.height = cameraInfo.height;
  camera_info_.width  = cameraInfo.width;  
  flag_update_camera_info = true;
}
