#pragma once
#ifndef _TNN_RECOGNIZER_H_
#define _TNN_RECOGNIZER_H_

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

#include "../../object/TNN/TNN_Recognize.h"

using namespace std;
using namespace ros;
using namespace cv;

class TNN_Recognizer
{    
  public:
    string nodeName = "TNN_Recognizer";
    string topic_image_sub = "TNN_Recognizer/image";
    string topic_image_recognize_pub = "TNN_Recognizer/image_recognize";
    string topic_label_pub = "TNN_Recognizer/label";
    string topic_accuracy_pub = "TNN_Recognizer/accuracy";
    
  private:
    string ver_ = "1.1";
    int queue_size = 4;
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_recognize_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image_recognize;
    cv_bridge::CvImageConstPtr/*CvImagePtr*/ cv_ptr;
    image_transport::SubscriberStatusCallback connect_cb_image_recognize;
    image_transport::SubscriberStatusCallback disconnect_cb_image_recognize;
    ros::SubscriberStatusCallback connect_cb_label;
    ros::SubscriberStatusCallback disconnect_cb_label;
      
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_recognize_pub_;
    ros::Publisher label_pub_;
    ros::Publisher accuracy_pub_;
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void connectCb_image_recognize(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_recognize_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_recognize_pub.c_str());
      flag_image_recognize = true;
      if(!(flag_image_recognize & flag_label))
      {
        sub_init();
      }
    }
    void disconnectCb_image_recognize(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_recognize_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_recognize_pub.c_str());
      flag_image_recognize = false;
      if(!(flag_image_recognize | flag_label))
      {
        flag_motion_init = false;
        sub_shutdown();
      }
    }
    void connectCb_label(const ros::SingleSubscriberPublisher& ssp)
    {
      if(accuracy_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_label_pub.c_str());
      flag_label = true;
      if(!(flag_image_recognize & flag_label))
      {
        sub_init();
      }
    }
    void disconnectCb_label(const ros::SingleSubscriberPublisher&)
    {
      if(accuracy_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_label_pub.c_str());
      flag_label = false;
      if(!(flag_image_recognize | flag_label))
      {
        flag_motion_init = false;
        sub_shutdown();
      }
    }
    
  private:
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_recognize_publish();
    void accuracy_publish();
    void label_publish();
    
  private:
    TNN_Recognize *tnn_recognize;
    Mat image;
    Mat image_recognize;
    bool flag_image_recognize = false;
    bool flag_label = false;
    bool flag_accuracy = false;
    

  public:
    TNN_Recognizer(ros::NodeHandle& nh);
    ~TNN_Recognizer();
    void run();
    
  public:
    virtual void init()
    {
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_recognize_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_recognize = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_recognize    = boost::bind(&TNN_Recognizer::connectCb_image_recognize, this, _1);
      disconnect_cb_image_recognize = boost::bind(&TNN_Recognizer::disconnectCb_image_recognize, this, _1);
      connect_cb_label    = boost::bind(&TNN_Recognizer::connectCb_label, this, _1);
      disconnect_cb_label = boost::bind(&TNN_Recognizer::disconnectCb_label, this, _1);
      pub_init();
      pub_topic_get();
    }
    
};

TNN_Recognizer::TNN_Recognizer(ros::NodeHandle& nh) : n_(nh)
{    
    tnn_recognize = new TNN_Recognize;
}

TNN_Recognizer::~TNN_Recognizer()
{
    delete tnn_recognize;
}

void TNN_Recognizer::run()
{  
    if(tnn_recognize -> run(image_recognize))
    {
      label_publish();
      accuracy_publish();
    }
}

void TNN_Recognizer::init()
{
    ROS_INFO("TNN_Recognizer initializing ...");
    if(tnn_recognize != NULL)
    {
        delete tnn_recognize;
        tnn_recognize = new tnn_recognize;
    }
    ROS_INFO("TNN_Recognizer constructing network ...");
    tnn_recognize -> construct_net();
    ROS_INFO("TNN_Recognizer weight loading ...");
    if(tnn_recognize -> weight_load())
    {}

    ROS_INFO("-- defalut weight path : %s",tnn_recognize -> tnn_fileName);
}

void TNN_Recognizer::image_recognize_publish()
{
  if(!flag_image_recognize) return;
  msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_recognize).toImageMsg();
  it_recognize_pub_.publish(msg_image);
}

void TNN_Recognizer::image_callBack(const sensor_msgs::ImageConstPtr& msg)
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

void TNN_Recognizer::label_publish()
{
    if(!flag_label) return;
    std_msgs:: msg;
}
    
void TNN_Recognizer::pub_topic_get()
{
    topic_image_recognize_pub = it_recognize_pub_.getTopic();
    topic_accuracy_pub = accuracy_pub_.getTopic();
}

void TNN_Recognizer::pub_init()
{
    ROS_INFO("Publisher %s initiating !", topic_image_sub.c_str());
    it_recognize_pub_ = it_recognize_ -> advertise(topic_image_recognize_pub, queue_size, connect_cb_image_recognize, disconnect_cb_image_recognize);
    ROS_INFO("Publisher %s initiating !", topic_accuracy_pub.c_str());
    accuracy_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_accuracy_pub.c_str(), queue_size, connect_cb_label, disconnect_cb_label);
}

void TNN_Recognizer::pub_shutdown()
{
    ROS_WARN("Publisher %s shuting down !", topic_image_recognize_pub.c_str());
    it_recognize_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_accuracy_pub.c_str());
    accuracy_pub_.shutdown();
}

void TNN_Recognizer::sub_topic_get()
{   
    topic_image_sub = it_sub_.getTopic();
    topic_min_box_length_sub = min_box_length_sub_.getTopic();
    topic_man_box_length_sub = max_box_length_sub_.getTopic();
    topic_auto_adjust_sub = auto_adjust_sub_.getTopic();
    sub_manual_topic_get();
}

void TNN_Recognizer::sub_init()
{
    ROS_INFO("Subscriber %s initiating !", topic_image_sub.c_str());
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &TNN_Recognizer::image_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_min_box_length_sub.c_str());
    min_box_length_sub_ = n_.subscribe(topic_min_box_length_sub.c_str(), queue_size, &TNN_Recognizer::min_box_length_ratio_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_man_box_length_sub.c_str());
    max_box_length_sub_ = n_.subscribe(topic_man_box_length_sub.c_str(), queue_size, &TNN_Recognizer::max_box_length_ratio_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_auto_adjust_sub.c_str());
    auto_adjust_sub_ = n_.subscribe(topic_auto_adjust_sub.c_str(), queue_size, &TNN_Recognizer::auto_adjust_callBack, this);
    sub_manual_init();
}

void TNN_Recognizer::sub_shutdown()
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

#endif

