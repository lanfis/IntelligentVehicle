#pragma once
#ifndef _MSER_DETECTOR_H_
#define _MSER_DETECTOR_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
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

#include "../../object/MSER/MSER.h"
//#include "../../matrix/config/console_format.h"

using namespace std;
using namespace ros;
using namespace cv;

class MSER_Detector
{    
  public:
    string nodeName = "MSER_Detector";
    string topic_image_sub = "MSER_Detector/image";
    string topic_image_motion_pub = "MSER_Detector/image_motion";
    string topic_min_box_length_sub = "MSER_Detector/min_box_length";
    string topic_man_box_length_sub = "MSER_Detector/man_box_length";
    string topic_auto_adjust_sub = "MSER_Detector/auto_adjust";
    string topic_history_sub = "MSER_Detector/history";
    string topic_dist2Threshold_sub = "MSER_Detector/dist2Threshold";
    string topic_detectShadows_sub = "MSER_Detector/detectShadows";
    string topic_learning_rate_sub = "MSER_Detector/learning_rate";
    string topic_motion_roi_pub = "MSER_Detector/motion_roi";
    
  private:
    string ver_ = "1.1";
    int queue_size = 4;
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_mser_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image_mser;
    cv_bridge::CvImageConstPtr/*CvImagePtr*/ cv_ptr;
    image_transport::SubscriberStatusCallback connect_cb_image_mser;
    image_transport::SubscriberStatusCallback disconnect_cb_image_mser;
    ros::SubscriberStatusCallback connect_cb_mser_roi;
    ros::SubscriberStatusCallback disconnect_cb_mser_roi;
      
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_mser_pub_;
    ros::Subscriber min_box_length_sub_;
    ros::Subscriber max_box_length_sub_;
    ros::Subscriber auto_adjust_sub_;
    ros::Subscriber history_sub_;
    ros::Subscriber dist2Threshold_sub_;
    ros::Subscriber detectShadows_sub_;
    ros::Subscriber learning_rate_sub_;
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
      if(it_mser_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_motion_pub.c_str());
      flag_motion_image = true;
      if(!(flag_motion_image & flag_motion_roi))
      {
        sub_init();
      }
    }
    void disconnectCb_image_motion(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_mser_pub_.getNumSubscribers() > 0) return;
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
    void min_box_length_ratio_callBack(const std_msgs::Float32::ConstPtr& msg);
    void max_box_length_ratio_callBack(const std_msgs::Float32::ConstPtr& msg);
    void auto_adjust_callBack(const std_msgs::Bool::ConstPtr& msg);
    void history_callBack(const std_msgs::Int32::ConstPtr& msg);
    void dist2Threshold_callBack(const std_msgs::Float32::ConstPtr& msg);
    void detectShadows_callBack(const std_msgs::Bool::ConstPtr& msg);
    void learning_rate_callBack(const std_msgs::Float32::ConstPtr& msg);
    void roi_publish();
    
  private:
    MSER *mser;
    Mat image;
    Mat image_motion;
    vector<Rect> motion_box;
    bool flag_motion_image = false;
    bool flag_motion_roi = false;
    bool flag_motion_init = false;

  public:
    MSER_Detector(ros::NodeHandle& nh);
    ~MSER_Detector();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_mser_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_mser = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_mser    = boost::bind(&MSER_Detector::connectCb_image_motion, this, _1);
      disconnect_cb_image_mser = boost::bind(&MSER_Detector::disconnectCb_image_motion, this, _1);
      connect_cb_mser_roi    = boost::bind(&MSER_Detector::connectCb_motion_roi, this, _1);
      disconnect_cb_mser_roi = boost::bind(&MSER_Detector::disconnectCb_motion_roi, this, _1);
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
    }
    
};

MSER_Detector::MSER_Detector(ros::NodeHandle& nh) : n_(nh)
{    
    mser = new MSER;
}

MSER_Detector::~MSER_Detector()
{
    delete mser;
}

void MSER_Detector::run()
{  
    if(!flag_motion_init)
    {
        mser -> init();
        flag_motion_init = true;
    }
    mser -> run(this -> image, this -> image_motion, motion_box);
    //image.copyTo(image_motion);
    /*for(int i = 0; i < motion_box.size(); i++)
    {
       rectangle( image, motion_box[i], cv::Scalar(0, 255, 0), 2, 8, 0 );
    }*/
    roi_publish();
    motion_box.clear();
    image_motion_publish();
}


void MSER_Detector::image_motion_publish()
{
  if(!flag_motion_image) return;
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_motion).toImageMsg();
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
  it_mser_pub_.publish(msg_image);
}

void MSER_Detector::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      //cv_ptr -> image.copyTo(image);
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

void MSER_Detector::min_box_length_ratio_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Motion min_box_length is changed to %f !", msg -> data);
    mser -> min_box_length_ratio = msg -> data;
}

void MSER_Detector::max_box_length_ratio_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Motion max_box_length is changed to %f !", msg -> data);
    mser -> max_box_length_ratio = msg -> data;
}

void MSER_Detector::auto_adjust_callBack(const std_msgs::Bool::ConstPtr& msg)
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
    mser -> auto_adjust = msg -> data;
}

void MSER_Detector::history_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Motion history is changed to %d !", msg -> data);
    mser -> history = msg -> data;
    flag_motion_init = false;
}

void MSER_Detector::dist2Threshold_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Motion dist2Threshold is changed to %f !", msg -> data);
    mser -> dist2Threshold = msg -> data;
    flag_motion_init = false;
}

void MSER_Detector::detectShadows_callBack(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg -> data)
      ROS_INFO("Motion detectShadows on !");
	else
      ROS_INFO("Motion detectShadows off !");
    mser -> detectShadows = msg -> data;
	flag_motion_init = false;
}

void MSER_Detector::learning_rate_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Motion learning_rate is changed to %f!", msg -> data);
    mser -> learning_rate = msg -> data;
}

void MSER_Detector::roi_publish()
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
    
void MSER_Detector::pub_topic_get()
{
    topic_image_motion_pub = it_mser_pub_.getTopic();
    topic_motion_roi_pub = motion_roi_pub_.getTopic();
}

void MSER_Detector::pub_init()
{
    ROS_INFO("Publisher %s initiating !", topic_image_sub.c_str());
    it_mser_pub_ = it_mser_ -> advertise(topic_image_motion_pub, queue_size, connect_cb_image_mser, disconnect_cb_image_mser);
    ROS_INFO("Publisher %s initiating !", topic_motion_roi_pub.c_str());
    motion_roi_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_motion_roi_pub.c_str(), queue_size, connect_cb_mser_roi, disconnect_cb_mser_roi);
}

void MSER_Detector::pub_shutdown()
{
    ROS_WARN("Publisher %s shuting down !", topic_image_motion_pub.c_str());
    it_mser_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_motion_roi_pub.c_str());
    motion_roi_pub_.shutdown();
}

void MSER_Detector::sub_topic_get()
{   
    topic_image_sub = it_sub_.getTopic();
    topic_min_box_length_sub = min_box_length_sub_.getTopic();
    topic_man_box_length_sub = max_box_length_sub_.getTopic();
    topic_auto_adjust_sub = auto_adjust_sub_.getTopic();
    sub_manual_topic_get();
}

void MSER_Detector::sub_init()
{
    ROS_INFO("Subscriber %s initiating !", topic_image_sub.c_str());
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &MSER_Detector::image_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_min_box_length_sub.c_str());
    min_box_length_sub_ = n_.subscribe(topic_min_box_length_sub.c_str(), queue_size, &MSER_Detector::min_box_length_ratio_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_man_box_length_sub.c_str());
    max_box_length_sub_ = n_.subscribe(topic_man_box_length_sub.c_str(), queue_size, &MSER_Detector::max_box_length_ratio_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_auto_adjust_sub.c_str());
    auto_adjust_sub_ = n_.subscribe(topic_auto_adjust_sub.c_str(), queue_size, &MSER_Detector::auto_adjust_callBack, this);
    sub_manual_init();
}

void MSER_Detector::sub_shutdown()
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

void MSER_Detector::sub_manual_topic_get()
{   
    if(mser -> auto_adjust) return;
    topic_history_sub = history_sub_.getTopic();
    topic_dist2Threshold_sub = dist2Threshold_sub_.getTopic();
    topic_detectShadows_sub = detectShadows_sub_.getTopic();
    topic_learning_rate_sub = learning_rate_sub_.getTopic();
}

void MSER_Detector::sub_manual_init()
{
    if(mser -> auto_adjust) return;
    ROS_INFO("Subscriber %s initiating !", topic_history_sub.c_str());
    history_sub_ = n_.subscribe(topic_history_sub.c_str(), queue_size, &MSER_Detector::history_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_dist2Threshold_sub.c_str());
    dist2Threshold_sub_ = n_.subscribe(topic_dist2Threshold_sub.c_str(), queue_size, &MSER_Detector::dist2Threshold_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_detectShadows_sub.c_str());
    detectShadows_sub_ = n_.subscribe(topic_detectShadows_sub.c_str(), queue_size, &MSER_Detector::detectShadows_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_learning_rate_sub.c_str());
    learning_rate_sub_ = n_.subscribe(topic_learning_rate_sub.c_str(), queue_size, &MSER_Detector::learning_rate_callBack, this);
}

void MSER_Detector::sub_manual_shutdown()
{
    if(mser -> auto_adjust) return;
    ROS_WARN("Subscriber %s shuting down !", topic_history_sub.c_str());
    history_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_dist2Threshold_sub.c_str());
    dist2Threshold_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_detectShadows_sub.c_str());
    detectShadows_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_learning_rate_sub.c_str());
    learning_rate_sub_.shutdown();
}
#endif

