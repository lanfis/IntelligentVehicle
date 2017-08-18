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
    string topic_image_cars_pub = "Face_Detector/image_cars";
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
    ros::SubscriberStatusCallback connect_cb_image_face;
    ros::SubscriberStatusCallback disconnect_cb_image_face;
    ros::SubscriberStatusCallback connect_cb_image_fullbody;
    ros::SubscriberStatusCallback disconnect_cb_image_fullbody;
    ros::SubscriberStatusCallback connect_cb_image_cars;
    ros::SubscriberStatusCallback disconnect_cb_image_cars;
      
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_detect_pub_;
    ros::Publisher image_face_pub_;
    ros::Publisher image_fullbody_pub_;
    ros::Publisher image_cars_pub_;
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
      if(!flag_face_image)
      {
        flag_face_image = true;
        if(!(flag_image_face | flag_image_fullbody | flag_image_cars))
        {
          sub_init();
          flag_image_face = true;//extra parameter
          flag_image_cars = true;//extra parameter
        }
      }
    }
    void disconnectCb_image_detect(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_detect_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_detect_pub.c_str());
      if(flag_face_image)
      {
        flag_face_image = false;
        flag_image_face = false;//extra parameter
        flag_image_cars = false;//extra parameter
        if(!(flag_image_face | flag_image_fullbody | flag_image_cars))
        {
          sub_shutdown();
        }
      }
    }
    void connectCb_image_face(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_face_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_face_pub.c_str());
      if(!flag_image_face)
      {
        flag_image_face = true;
        if(!(flag_face_image | flag_image_fullbody | flag_image_cars))
        {
          sub_init();
        }
      }
    }
    void disconnectCb_image_face(const ros::SingleSubscriberPublisher&)
    {
      if(image_face_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_face_pub.c_str());
      if(flag_image_face)
      {
        flag_image_face = false;
        if(!(flag_face_image | flag_image_fullbody | flag_image_cars))
        {
          sub_shutdown();
        }
      }
    }
    void connectCb_image_fullbody(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_fullbody_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_fullbody_pub.c_str());
      if(!flag_image_fullbody)
      {
        flag_image_fullbody = true;
        if(!(flag_face_image | flag_image_face | flag_image_cars))
        {
          sub_init();
        }
      }
    }
    void disconnectCb_image_fullbody(const ros::SingleSubscriberPublisher&)
    {
      if(image_fullbody_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_fullbody_pub.c_str());
      if(flag_image_fullbody)
      {
        flag_image_fullbody = false;
        if(!(flag_face_image | flag_image_face | flag_image_cars))
        {
          sub_shutdown();
        }
      }
    }
    void connectCb_image_cars(const ros::SingleSubscriberPublisher& ssp)
    {
      if(image_cars_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_cars_pub.c_str());
      if(!flag_image_cars)
      {
        flag_image_cars = true;
        if(!(flag_face_image | flag_image_face | flag_image_fullbody))
        {
          sub_init();
        }
      }
    }
    void disconnectCb_image_cars(const ros::SingleSubscriberPublisher&)
    {
      if(image_cars_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_cars_pub.c_str());
      if(flag_image_cars)
      {
        flag_image_cars = false;
        if(!(flag_face_image | flag_image_face | flag_image_fullbody))
        {
          sub_shutdown();
        }
      }
    }
    
  private:
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_detect_publish();
    void image_face_publish();
    void image_fullbody_publish();
    void image_cars_publish();
    
  private:
    Face_Detector_Cascade *fdc;
    Mat image;
    bool activation_image_face = true;
    bool activation_image_fullbody = false;
    bool activation_image_cars = true;
    bool flag_face_image = false;
    bool flag_image_face = false;
    bool flag_image_fullbody = false;
    bool flag_image_cars = false;

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
      connect_cb_image_face    = boost::bind(&Face_Detector::connectCb_image_face, this, _1);
      disconnect_cb_image_face = boost::bind(&Face_Detector::disconnectCb_image_face, this, _1);
      connect_cb_image_fullbody    = boost::bind(&Face_Detector::connectCb_image_fullbody, this, _1);
      disconnect_cb_image_fullbody = boost::bind(&Face_Detector::disconnectCb_image_fullbody, this, _1);
      connect_cb_image_cars    = boost::bind(&Face_Detector::connectCb_image_cars, this, _1);
      disconnect_cb_image_cars = boost::bind(&Face_Detector::disconnectCb_image_cars, this, _1);
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
    if(activation_image_fullbody & flag_image_fullbody) fdc -> fullbody_detect(this -> image);
    if(activation_image_cars & flag_image_cars) fdc -> cars_detect(this -> image);
    if(activation_image_face & flag_image_face) fdc -> face_detect(this -> image);
    for(int i = 0; i < fdc -> fullbody.size(); i++)
    {
       rectangle( image, fdc -> fullbody[i], cv::Scalar(0, 255, 0), 1, 8, 0 );
    }
    for(int i = 0; i < fdc -> cars.size(); i++)
    {
       rectangle( image, fdc -> cars[i], cv::Scalar(0, 255, 255), 1, 8, 0 );
    }
    for(int i = 0; i < fdc -> face.size(); i++)
    {
       rectangle( image, fdc -> face[i], cv::Scalar(0, 0, 255), 1, 8, 0 );
    }
    image_detect_publish();
    image_face_publish();
    image_fullbody_publish();
    image_cars_publish();
}

void Face_Detector::image_detect_publish()
{
  if(!flag_face_image) return;
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

void Face_Detector::image_face_publish()
{
    if(!flag_image_face) return;
    sensor_msgs::RegionOfInterest msg;
    for(int i = 0; i < fdc -> face.size(); i++)
    {
      msg.x_offset = fdc -> face[i].x + fdc -> face[i].width/2;      
      msg.y_offset = fdc -> face[i].y + fdc -> face[i].height/2;      
      msg.width    = fdc -> face[i].width;      
      msg.height   = fdc -> face[i].height;     
      image_face_pub_.publish(msg);
    }
}

void Face_Detector::image_fullbody_publish()
{
    if(!flag_image_fullbody) return;
    sensor_msgs::RegionOfInterest msg;
    for(int i = 0; i < fdc -> fullbody.size(); i++)
    {
      msg.x_offset = fdc -> fullbody[i].x + fdc -> fullbody[i].width/2;      
      msg.y_offset = fdc -> fullbody[i].y + fdc -> fullbody[i].height/2;      
      msg.width    = fdc -> fullbody[i].width;      
      msg.height   = fdc -> fullbody[i].height;     
      image_fullbody_pub_.publish(msg);
    }
}

void Face_Detector::image_cars_publish()
{
    if(!flag_image_cars) return;
    sensor_msgs::RegionOfInterest msg;
    for(int i = 0; i < fdc -> fullbody.size(); i++)
    {
      msg.x_offset = fdc -> cars[i].x + fdc -> cars[i].width/2;      
      msg.y_offset = fdc -> cars[i].y + fdc -> cars[i].height/2;      
      msg.width    = fdc -> cars[i].width;      
      msg.height   = fdc -> cars[i].height;     
      image_cars_pub_.publish(msg);
    }
}
    
void Face_Detector::pub_topic_get()
{
    topic_image_detect_pub = it_detect_pub_.getTopic();
    topic_image_face_pub = image_face_pub_.getTopic();
    topic_image_fullbody_pub = image_fullbody_pub_.getTopic();
    topic_image_cars_pub = image_cars_pub_.getTopic();
}

void Face_Detector::pub_init()
{
    ROS_INFO("Publisher %s initiating !", topic_image_detect_pub.c_str());
    it_detect_pub_ = it_detect_ -> advertise(topic_image_detect_pub, queue_size, connect_cb_image_detect, disconnect_cb_image_detect);
    ROS_INFO("Publisher %s initiating !", topic_image_face_pub.c_str());
    image_face_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_image_face_pub.c_str(), queue_size, connect_cb_image_face, disconnect_cb_image_face);
    ROS_INFO("Publisher %s initiating !", topic_image_fullbody_pub.c_str());
    image_fullbody_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_image_fullbody_pub.c_str(), queue_size, connect_cb_image_fullbody, disconnect_cb_image_fullbody);
    ROS_INFO("Publisher %s initiating !", topic_image_cars_pub.c_str());
    image_cars_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_image_cars_pub.c_str(), queue_size, connect_cb_image_cars, disconnect_cb_image_cars);
}

void Face_Detector::pub_shutdown()
{
    ROS_WARN("Publisher %s shuting down !", topic_image_detect_pub.c_str());
    it_detect_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_image_face_pub.c_str());
    image_face_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_image_fullbody_pub.c_str());
    image_fullbody_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_image_cars_pub.c_str());
    image_cars_pub_.shutdown();
}

void Face_Detector::sub_topic_get()
{   
    topic_image_sub = it_sub_.getTopic();
}

void Face_Detector::sub_init()
{
    ROS_INFO("Subscriber %s initiating !", topic_image_sub.c_str());
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &Face_Detector::image_callBack, this);
}

void Face_Detector::sub_shutdown()
{
    ROS_WARN("Subscriber %s shuting down !", topic_image_sub.c_str());
    it_sub_.shutdown();
}

#endif

