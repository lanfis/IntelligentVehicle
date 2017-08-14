#pragma once
#ifndef _FACE_DETECTOR_H_
#define _FACE_DETECTOR_H_

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

#include "../../object/FACE/Face_Detector_Cascade.h"

using namespace std;
using namespace ros;
using namespace cv;

class Face_Detector
{    
  public:
    string nodeName = "Face_Detector";
    string topic_image_sub = "Face_Detector/image";
	string topic_image_detect_pub = "Face_Detector/image_detect";
    string topic_image_face_pub = "Face_Detector/image_face";
    string topic_image_fullbody_pub = "Face_Detector/image_fullbody";
    string topic_image_upperbody_pub = "Face_Detector/image_upperbody";
    //string topic_auto_adjust_sub = "Face_Detector/auto_adjust";
    
  private:
    string ver_ = "1.1";
    int queue_size = 4;
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_detect_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image_detect;
    cv_bridge::CvImageConstPtr/*CvImagePtr*/ cv_ptr;
    image_transport::SubscriberStatusCallback connect_cb_image_detect;
    image_transport::SubscriberStatusCallback disconnect_cb_image_detect;
    ros::SubscriberStatusCallback connect_cb_face;
    ros::SubscriberStatusCallback disconnect_cb_face;
    ros::SubscriberStatusCallback connect_cb_fullbody;
    ros::SubscriberStatusCallback disconnect_cb_fullbody;
    ros::SubscriberStatusCallback connect_cb_upperbody;
    ros::SubscriberStatusCallback disconnect_cb_upperbody;
      
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_detect_pub_;
    ros::Publisher image_face_pub_;
    ros::Publisher image_fullbody_pub_;
    ros::Publisher image_upperbody_pub_;
    //ros::Subscriber auto_adjust_sub_;
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    /*void sub_manual_topic_get();
    void sub_manual_init();
    void sub_manual_shutdown();*/
    void connectCb_image_detect(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_detect_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_detect_pub.c_str());
      flag_motion_image = true;
      if(!(flag_motion_image & flag_motion_roi))
      {
        sub_init();
      }
    }
    void disconnectCb_image_detect(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_detect_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_detect_pub.c_str());
      flag_motion_image = false;
      if(!(flag_motion_image | flag_motion_roi))
      {
        flag_motion_init = false;
        sub_shutdown();
      }
    }
    void connectCb_image_face(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_face_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_face_pub.c_str());
      flag_motion_roi = true;
      if(!(flag_motion_image & flag_motion_roi))
      {
        sub_init();
      }
    }
    void disconnectCb_image_face(const ros::SingleSubscriberPublisher&)
    {
      if(image_face_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_face_pub.c_str());
      flag_motion_roi = false;
      if(!(flag_motion_image | flag_motion_roi))
      {
        flag_motion_init = false;
        sub_shutdown();
      }
    }
    void connectCb_image_fullbody(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_fullbody_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_fullbody_pub.c_str());
      flag_motion_roi = true;
      if(!(flag_motion_image & flag_motion_roi))
      {
        sub_init();
      }
    }
    void disconnectCb_image_fullbody(const ros::SingleSubscriberPublisher&)
    {
      if(image_fullbody_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_fullbody_pub.c_str());
      flag_motion_roi = false;
      if(!(flag_motion_image | flag_motion_roi))
      {
        flag_motion_init = false;
        sub_shutdown();
      }
    }
    void connectCb_image_upperbody(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_upperbody_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_upperbody_pub.c_str());
      flag_motion_roi = true;
      if(!(flag_motion_image & flag_motion_roi))
      {
        sub_init();
      }
    }
    void disconnectCb_image_upperbody(const ros::SingleSubscriberPublisher&)
    {
      if(image_upperbody_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_upperbody_pub.c_str());
      flag_motion_roi = false;
      if(!(flag_motion_image | flag_motion_roi))
      {
        flag_motion_init = false;
        sub_shutdown();
      }
    }
    
  private:
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_detect_publish();
    void image_face_publish();
    void image_fullbody_publish();
    void image_upperbody_publish();
    
  private:
    Face_Detector_Cascade *fdc;
    Mat image;
    bool flag_motion_image = false;
    bool flag_motion_roi = false;
    bool flag_motion_init = false;

  public:
    Face_Detector(ros::NodeHandle& nh);
    ~Face_Detector();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_detect_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_detect = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_detect    = boost::bind(&Face_Detector::connectCb_image_detect, this, _1);
      disconnect_cb_image_detect = boost::bind(&Face_Detector::disconnectCb_image_detect, this, _1);
      connect_cb_face    = boost::bind(&Face_Detector::connectCb_image_face, this, _1);
      disconnect_cb_face = boost::bind(&Face_Detector::disconnectCb_image_face, this, _1);
      connect_cb_fullbody    = boost::bind(&Face_Detector::connectCb_image_fullbody, this, _1);
      disconnect_cb_fullbody = boost::bind(&Face_Detector::disconnectCb_image_fullbody, this, _1);
      connect_cb_upperbody    = boost::bind(&Face_Detector::connectCb_image_upperbody, this, _1);
      disconnect_cb_upperbody = boost::bind(&Face_Detector::disconnectCb_image_upperbody, this, _1);
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
    }
    
};

Face_Detector::Face_Detector(ros::NodeHandle& nh) : n_(nh)
{    
    fdc = new Face_Detector_Cascade;
}

Face_Detector::~Face_Detector()
{
    delete fdc;
}

void Face_Detector::run()
{  
    fdc -> fullbody_detect(this -> image);
    fdc -> upperbody_detect(this -> image);
    fdc -> face_detect(this -> image);
    for(int i = 0; i < motion_box.size(); i++)
    {
       rectangle( image, fdc -> fullbody[i], cv::Scalar(0, 255, 0), 1, 8, 0 );
    }
}

void Face_Detector::image_detect_publish()
{
  if(!flag_motion_image) return;
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
  it_detect_pub_.publish(msg_image);
}

void Face_Detector::image_callBack(const sensor_msgs::ImageConstPtr& msg)
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
}
/*
void Face_Detector::auto_adjust_callBack(const std_msgs::Bool::ConstPtr& msg)
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
    fdc -> auto_adjust = msg -> data;
}
*/
    void image_detect_publish();
    void image_face_publish();
    void image_fullbody_publish();
    void image_upperbody_publish();

void Face_Detector::image_fullbody_publish()
{
    if(!flag_motion_roi) return;
    sensor_msgs::RegionOfInterest msg;
    for(int i = 0; i < motion_box.size(); i++)
    {
      msg.x_offset = fdc -> fullbody[i].x + fdc -> fullbody[i].width/2;      
      msg.y_offset = fdc -> fullbody[i].y + fdc -> fullbody[i].height/2;      
      msg.width    = fdc -> fullbody[i].width;      
      msg.height   = fdc -> fullbody[i].height;     
      image_face_pub_.publish(msg);
    }
}
    
void Face_Detector::pub_topic_get()
{
    topic_image_face_pub = it_detect_pub_.getTopic();
    topic_motion_roi_pub = image_face_pub_.getTopic();
}

void Face_Detector::pub_init()
{
    ROS_INFO("Publisher %s initiating !", topic_image_sub.c_str());
    it_detect_pub_ = it_detect_ -> advertise(topic_image_face_pub, queue_size, connect_cb_image_detect, disconnect_cb_image_detect);
    ROS_INFO("Publisher %s initiating !", topic_motion_roi_pub.c_str());
    image_face_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_motion_roi_pub.c_str(), queue_size, connect_cb_face, disconnect_cb_face);
}

void Face_Detector::pub_shutdown()
{
    ROS_WARN("Publisher %s shuting down !", topic_image_face_pub.c_str());
    it_detect_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_motion_roi_pub.c_str());
    image_face_pub_.shutdown();
}

void Face_Detector::sub_topic_get()
{   
    topic_image_sub = it_sub_.getTopic();
    topic_image_fullbody_pub = image_face_pub_.getTopic();
    topic_image_upperbody_pub = image_fullbody_pub_.getTopic();
    topic_auto_adjust_sub = auto_adjust_sub_.getTopic();
    sub_manual_topic_get();
}

void Face_Detector::sub_init()
{
    ROS_INFO("Subscriber %s initiating !", topic_image_sub.c_str());
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &Face_Detector::image_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_image_fullbody_pub.c_str());
    image_face_pub_ = n_.subscribe(topic_image_fullbody_pub.c_str(), queue_size, &Face_Detector::min_box_length_ratio_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_image_upperbody_pub.c_str());
    image_fullbody_pub_ = n_.subscribe(topic_image_upperbody_pub.c_str(), queue_size, &Face_Detector::max_box_length_ratio_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_auto_adjust_sub.c_str());
    auto_adjust_sub_ = n_.subscribe(topic_auto_adjust_sub.c_str(), queue_size, &Face_Detector::auto_adjust_callBack, this);
    sub_manual_init();
}

void Face_Detector::sub_shutdown()
{
    ROS_WARN("Subscriber %s shuting down !", topic_image_sub.c_str());
    it_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_image_fullbody_pub.c_str());
    image_face_pub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_image_upperbody_pub.c_str());
    image_fullbody_pub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_auto_adjust_sub.c_str());
    auto_adjust_sub_.shutdown();
    sub_manual_shutdown();
}

void Face_Detector::sub_manual_topic_get()
{   
    if(fdc -> auto_adjust) return;
    topic_bg_filter_ksize_sub = bg_filter_ksize_sub_.getTopic();
    topic_bg_filter_sigma_sub = bg_filter_sigma_sub_.getTopic();
    topic_num_detect_sub = num_detect_sub_.getTopic();
    topic_num_detect_diff_sub = num_detect_diff_sub_.getTopic();
}

void Face_Detector::sub_manual_init()
{
    if(fdc -> auto_adjust) return;
    ROS_INFO("Subscriber %s initiating !", topic_bg_filter_ksize_sub.c_str());
    bg_filter_ksize_sub_ = n_.subscribe(topic_bg_filter_ksize_sub.c_str(), queue_size, &Face_Detector::bg_filter_ksize_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_bg_filter_sigma_sub.c_str());
    bg_filter_sigma_sub_ = n_.subscribe(topic_bg_filter_sigma_sub.c_str(), queue_size, &Face_Detector::bg_filter_sigma_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_num_detect_sub.c_str());
    num_detect_sub_ = n_.subscribe(topic_num_detect_sub.c_str(), queue_size, &Face_Detector::num_detect_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_num_detect_diff_sub.c_str());
    num_detect_diff_sub_ = n_.subscribe(topic_num_detect_diff_sub.c_str(), queue_size, &Face_Detector::num_detect_diff_callBack, this);
}

void Face_Detector::sub_manual_shutdown()
{
    if(fdc -> auto_adjust) return;
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

