#pragma once
#ifndef _MOTION_DETECTOR_H_
#define _MOTION_DETECTOR_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/RegionOfInterest.h>
/*
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/UInt16MultiArray.h>
*/

#include "../../object/MOTION/Motion.h"

using namespace std;
using namespace ros;
using namespace cv;

class Motion_Detector// : public nodelet::Nodelet
{    
  public:
    string nodeName = "Motion_Detector";
    string topic_image_sub = "Motion_Detector/image";
    string topic_image_motion_pub = "Motion_Detector/image_motion";
    string topic_min_box_length_sub = "Motion_Detector/min_box_length";
    string topic_man_box_length_sub = "Motion_Detector/man_box_length_sub";
    string topic_auto_adjust_sub = "Motion_Detector/auto_adjust";
    string topic_bg_filter_ksize_sub = "Motion_Detector/bg_filter_ksize";
    string topic_bg_filter_sigma_sub = "Motion_Detector/bg_filter_sigma";
    string topic_num_detect_sub = "Motion_Detector/num_detect";
    string topic_num_detect_diff_sub = "Motion_Detector/num_detect_diff";
	string topic_motion_roi_pub = "Motion_Detector/motion_roi";
    
  private:
    string ver_ = "1.1";
    int queue_size = 4;
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_motion_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image_motion;
    cv_bridge::CvImageConstPtr/*CvImagePtr*/ cv_ptr;
    image_transport::SubscriberStatusCallback connect_cb_image_motion;
    image_transport::SubscriberStatusCallback disconnect_cb_image_motion;
    ros::SubscriberStatusCallback connect_cb_motion_roi;
    ros::SubscriberStatusCallback disconnect_cb_motion_roi;
      
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_motion_pub_;
    ros::Subscriber min_box_length_sub_;
    ros::Subscriber max_box_length_sub_;
    ros::Subscriber auto_adjust_sub_;
    ros::Subscriber bg_filter_ksize_sub_;
    ros::Subscriber bg_filter_sigma_sub_;
    ros::Subscriber num_detect_sub_;
    ros::Subscriber num_detect_diff_sub_;
	ros::Publisher motion_roi_pub_;
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void sub_manual_topic_get();
    void sub_manual_init();
    void sub_manual_shutdown();
    void connectCb_image_motion(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_motion_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_motion_pub.c_str());
      flag_motion_image = true;
	  if(!(flag_motion_image & flag_motion_roi))
	  {
        sub_init();
	  }
    }
    void disconnectCb_image_motion(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_motion_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_motion_pub.c_str());
      flag_motion_image = false;
	  if(!(flag_motion_image | flag_motion_roi))
	  {
        flag_motion_init = false;
        sub_shutdown();
	  }
    }
    void connectCb_motion_roi(const ros::SingleSubscriberPublisher& ssp)
    {
      if(motion_roi_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_motion_roi_pub.c_str());
      flag_motion_roi = true;
	  if(!(flag_motion_image & flag_motion_roi))
	  {
        sub_init();
	  }
    }
    void disconnectCb_motion_roi(const ros::SingleSubscriberPublisher&)
    {
      if(motion_roi_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_motion_roi_pub.c_str());
      flag_motion_roi = false;
	  if(!(flag_motion_image | flag_motion_roi))
	  {
        flag_motion_init = false;
        sub_shutdown();
	  }
    }
    
  private:
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_motion_publish();
    void min_box_length_callBack(const std_msgs::Int32::ConstPtr& msg);
    void max_box_length_callBack(const std_msgs::Int32::ConstPtr& msg);
    void auto_adjust_callBack(const std_msgs::Bool::ConstPtr& msg);
    void bg_filter_ksize_callBack(const std_msgs::Int32::ConstPtr& msg);
    void bg_filter_sigma_callBack(const std_msgs::Float32::ConstPtr& msg);
    void num_detect_callBack(const std_msgs::Int32::ConstPtr& msg);
    void num_detect_diff_callBack(const std_msgs::Int32::ConstPtr& msg);
	void roi_publish();
    
  private:
    MOTION *motion;
    Mat image;
    Mat image_motion;
    vector<Rect> motion_box;
    int motion_sensity_level = 5;
    bool flag_motion_image = false;
	bool flag_motion_roi = false;
    bool flag_motion_init = false;
    /*
    double min_box_length;
    double max_box_length;
    bool auto_adjust;
    //NOT IN USE
    int bg_filter_ksize;
    double bg_filter_sigma;
    int num_detect;
    int num_detect_tor;
    int num_detect_diff;
    //NOT IN USE
    */

  public:
    Motion_Detector(ros::NodeHandle& nh);
    ~Motion_Detector();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_motion_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_motion = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_motion    = boost::bind(&Motion_Detector::connectCb_image_motion, this, _1);
      disconnect_cb_image_motion = boost::bind(&Motion_Detector::disconnectCb_image_motion, this, _1);
      connect_cb_motion_roi    = boost::bind(&Motion_Detector::connectCb_motion_roi, this, _1);
      disconnect_cb_motion_roi = boost::bind(&Motion_Detector::disconnectCb_motion_roi, this, _1);
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
    }
    
};

Motion_Detector::Motion_Detector(ros::NodeHandle& nh) : n_(nh)
{    
    motion = new MOTION;
}

Motion_Detector::~Motion_Detector()
{
    delete motion;
}

void Motion_Detector::run()
{  
    if(!flag_motion_init)
    {
        motion -> init(this -> image);
        flag_motion_init = true;
    }
    motion -> run(this -> image, motion_box);
    //image.copyTo(image_motion);
    for(int i = 0; i < motion_box.size(); i++)
    {
       rectangle( image, motion_box[i], cv::Scalar(0, motion -> thresh*motion_sensity_level, 0), 1, 8, 0 );
    }
	roi_publish();
    motion_box.clear();
    image_motion_publish();
}


void Motion_Detector::image_motion_publish()
{
  if(!flag_motion_image) return;
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
  it_motion_pub_.publish(msg_image);
}

void Motion_Detector::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      //cv_ptr -> image.copyTo(*image);
      this -> image = cv_ptr -> image;
      run();
      return;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //sensor_msgs::Image::Ptr cv_ptr = cv_bridge::CvImage(msg->header, msg->encoding, out_image).toImageMsg();
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //this -> image = cv_ptr -> image;
}

void Motion_Detector::min_box_length_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Motion min_box_length is changed to %d !", msg -> data);
    motion -> min_box_length = msg -> data;
}

void Motion_Detector::max_box_length_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Motion max_box_length is changed to %d !", msg -> data);
    motion -> max_box_length = msg -> data;
}

void Motion_Detector::auto_adjust_callBack(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg -> data)
    {
      ROS_INFO("Motion auto_adjust on !");
      sub_manual_init();
      sub_manual_topic_get();
    }
    else
    {
      ROS_INFO("Motion auto_adjust off !");
      sub_manual_shutdown();
    }
    motion -> auto_adjust = msg -> data;
}

void Motion_Detector::bg_filter_ksize_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Motion bg_filter_ksize is changed to %d !", msg -> data);
    motion -> bg_filter_ksize = msg -> data;
}

void Motion_Detector::bg_filter_sigma_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Motion bg_filter_sigma is changed to %f !", msg -> data);
    motion -> bg_filter_sigma = msg -> data;
}

void Motion_Detector::num_detect_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Motion num_detect is changed to %d !", msg -> data);
    motion -> num_detect = msg -> data;
}

void Motion_Detector::num_detect_diff_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Motion num_detect_diff is changed to %d !", msg -> data);
    motion -> num_detect_diff = msg -> data;
}

void Motion_Detector::roi_publish()
{
    if(!flag_motion_roi) return;
	sensor_msgs::RegionOfInterest msg;
	for(int i = 0; i < motion_box.size(); i++)
	{
      msg.x_offset = motion_box[i].x + motion_box[i].width/2;      
      msg.y_offset = motion_box[i].y + motion_box[i].height/2;      
      msg.width    = motion_box[i].width;      
      msg.height   = motion_box[i].height;     
      motion_roi_pub_.publish(msg);
	}
}
    
void Motion_Detector::pub_topic_get()
{
    topic_image_motion_pub = it_motion_pub_.getTopic();
	topic_motion_roi_pub = motion_roi_pub_.getTopic();
}

void Motion_Detector::pub_init()
{
    ROS_INFO("Publisher %s initiating !", topic_image_sub.c_str());
    it_motion_pub_ = it_motion_ -> advertise(topic_image_motion_pub, queue_size, connect_cb_image_motion, disconnect_cb_image_motion);
    ROS_INFO("Publisher %s initiating !", topic_motion_roi_pub.c_str());
	motion_roi_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_motion_roi_pub.c_str(), queue_size, connect_cb_motion_roi, disconnect_cb_motion_roi);
}

void Motion_Detector::pub_shutdown()
{
    ROS_WARN("Publisher %s shuting down !", topic_image_motion_pub.c_str());
    it_motion_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_motion_roi_pub.c_str());
	motion_roi_pub_.shutdown();
}

void Motion_Detector::sub_topic_get()
{   
    topic_image_sub = it_sub_.getTopic();
    topic_min_box_length_sub = min_box_length_sub_.getTopic();
    topic_man_box_length_sub = max_box_length_sub_.getTopic();
    topic_auto_adjust_sub = auto_adjust_sub_.getTopic();
    sub_manual_topic_get();
}

void Motion_Detector::sub_init()
{
    ROS_INFO("Subscriber %s initiating !", topic_image_sub.c_str());
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &Motion_Detector::image_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_min_box_length_sub.c_str());
    min_box_length_sub_ = n_.subscribe(topic_min_box_length_sub.c_str(), queue_size, &Motion_Detector::min_box_length_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_man_box_length_sub.c_str());
    max_box_length_sub_ = n_.subscribe(topic_man_box_length_sub.c_str(), queue_size, &Motion_Detector::max_box_length_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_auto_adjust_sub.c_str());
    auto_adjust_sub_ = n_.subscribe(topic_auto_adjust_sub.c_str(), queue_size, &Motion_Detector::auto_adjust_callBack, this);
    sub_manual_init();
}

void Motion_Detector::sub_shutdown()
{
    ROS_WARN("Subscriber %s shuting down !", topic_image_sub.c_str());
    it_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_min_box_length_sub.c_str());
    min_box_length_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_man_box_length_sub.c_str());
    max_box_length_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_auto_adjust_sub.c_str());
    auto_adjust_sub_.shutdown();
    sub_manual_shutdown();
}

void Motion_Detector::sub_manual_topic_get()
{   
    if(motion -> auto_adjust) return;
    topic_bg_filter_ksize_sub = bg_filter_ksize_sub_.getTopic();
    topic_bg_filter_sigma_sub = bg_filter_sigma_sub_.getTopic();
    topic_num_detect_sub = num_detect_sub_.getTopic();
    topic_num_detect_diff_sub = num_detect_diff_sub_.getTopic();
}

void Motion_Detector::sub_manual_init()
{
    if(motion -> auto_adjust) return;
    ROS_INFO("Subscriber %s initiating !", topic_bg_filter_ksize_sub.c_str());
    bg_filter_ksize_sub_ = n_.subscribe(topic_bg_filter_ksize_sub.c_str(), queue_size, &Motion_Detector::bg_filter_ksize_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_bg_filter_sigma_sub.c_str());
    bg_filter_sigma_sub_ = n_.subscribe(topic_bg_filter_sigma_sub.c_str(), queue_size, &Motion_Detector::bg_filter_sigma_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_num_detect_sub.c_str());
    num_detect_sub_ = n_.subscribe(topic_num_detect_sub.c_str(), queue_size, &Motion_Detector::num_detect_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_num_detect_diff_sub.c_str());
    num_detect_diff_sub_ = n_.subscribe(topic_num_detect_diff_sub.c_str(), queue_size, &Motion_Detector::num_detect_diff_callBack, this);
}

void Motion_Detector::sub_manual_shutdown()
{
    if(motion -> auto_adjust) return;
    ROS_WARN("Subscriber %s shuting down !", topic_bg_filter_ksize_sub.c_str());
    bg_filter_ksize_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_bg_filter_sigma_sub.c_str());
    bg_filter_sigma_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_num_detect_sub.c_str());
    num_detect_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_num_detect_diff_sub.c_str());
    num_detect_diff_sub_.shutdown();
}
#endif

