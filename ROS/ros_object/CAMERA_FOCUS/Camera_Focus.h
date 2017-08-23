#pragma once
#ifndef _CAMERA_FOCUS_H_
#define _CAMERA_FOCUS_H_

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
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/UInt16MultiArray.h>

#include "../../matrix/ros_link.h"

using namespace std;
using namespace ros;
using namespace cv;

class Camera_Focus// : public nodelet::Nodelet
{    
  public:
    string nodeName = "Camera_Focus";
    string topic_image_sub = "Camera_Focus/image";
    string topic_image_roi_pub = "Camera_Focus/image_roi";
    string topic_image_roi_ratio_sub = "Camera_Focus/image_roi_ratio";
    string topic_image_roi_width_offset_sub = "Camera_Focus/image_roi_width_offset";
    string topic_image_roi_height_offset_sub = "Camera_Focus/image_roi_height_offset";
    string topic_image_roi_resolution_sub = "Camera_Focus/image_roi_resolution";
    
  private:
    string ver_ = "1.1";
    int queue_size = 4;
	ROS_Link *ros_link;
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_roi_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image_roi;
    cv_bridge::CvImageConstPtr/*CvImagePtr*/ cv_ptr;
    image_transport::SubscriberStatusCallback connect_cb_image_roi;
    image_transport::SubscriberStatusCallback disconnect_cb_image_roi;
      
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_roi_pub_;
    ros::Subscriber roi_ratio_sub_;
    ros::Subscriber roi_width_offset_sub_;
    ros::Subscriber roi_height_offset_sub_;
    ros::Subscriber roi_resolution_sub_;
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void connectCb_image_roi(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_roi_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_roi_pub.c_str());
      sub_init();
    }
    void disconnectCb_image_roi(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_roi_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_roi_pub.c_str());
      sub_shutdown();
    }
    
  private:
    bool camera_init();
    bool camera_shutdown();
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_roi_publish();
    void roi_ratio_callBack(const std_msgs::Float32::ConstPtr& msg);
    void roi_width_offset_callBack(const std_msgs::Int32::ConstPtr& msg);
    void roi_height_offset_callBack(const std_msgs::Int32::ConstPtr& msg);
    void roi_resolution_callBack(const std_msgs::Float32::ConstPtr& msg);
    
  private:
    Mat image;
    Mat image_roi;
    float image_roi_ratio = 0.5;
    int image_roi_width_offset = 0;
    int image_roi_height_offset = 0;
    float image_roi_resolution = 1.0;
    
    void image_roi_get();

  public:
    Camera_Focus(ros::NodeHandle& nh);
    ~Camera_Focus();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_roi_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_roi = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_roi    = boost::bind(&Camera_Focus::connectCb_image_roi, this, _1);
      disconnect_cb_image_roi = boost::bind(&Camera_Focus::disconnectCb_image_roi, this, _1);
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
	  ros_link = new ROS_Link(n_, nodeName);
	  ros_link -> pub_init();
	  ros_link -> sub_init();
	  ros_link -> add_cell(roi_ratio_sub_, topic_image_roi_ratio_sub);
	  ros_link -> add_cell(roi_resolution_sub_, topic_image_roi_resolution_sub);
    }
    
};

Camera_Focus::Camera_Focus(ros::NodeHandle& nh) : n_(nh)
{    
}

Camera_Focus::~Camera_Focus()
{
}

void Camera_Focus::run()
{  
    image_roi_get();
    image_roi_publish();
}

void Camera_Focus::image_roi_get()
{
    int center_col = (image).cols/2;
    int center_row = (image).rows/2;
    int roi_width =  (image).cols*image_roi_ratio;
    int roi_height = (image).cols*image_roi_ratio;
    int col_s = ((center_col - roi_width/2  + image_roi_width_offset) < 0)? 0 : center_col - roi_width/2  + image_roi_width_offset;
    int col_e = ((center_col + roi_width/2  + image_roi_width_offset) > (image).cols)? (image).cols : center_col + roi_width/2  + image_roi_width_offset;
    int row_s = ((center_row - roi_height/2 + image_roi_height_offset)< 0)? 0 : center_row - roi_height/2 + image_roi_height_offset;
    int row_e = ((center_row + roi_height/2 + image_roi_height_offset)> (image).rows)? (image).rows : center_row + roi_height/2 + image_roi_height_offset;
    image_roi = (image)(Range(row_s, row_e), Range(col_s, col_e));
    resize(image_roi, image_roi, Size(image.cols*image_roi_resolution, image.rows*image_roi_resolution));
}

void Camera_Focus::image_roi_publish()
{
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_roi).toImageMsg();
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
  it_roi_pub_.publish(msg_image);
}

void Camera_Focus::image_callBack(const sensor_msgs::ImageConstPtr& msg)
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

void Camera_Focus::roi_ratio_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("ROI ratio is changed to %f !", msg -> data);
    image_roi_ratio = (msg -> data > 1.0)? 1.0 : msg -> data;
}

void Camera_Focus::roi_width_offset_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("ROI width offset is changed to %d !", msg -> data);
    image_roi_width_offset = msg -> data;
}

void Camera_Focus::roi_height_offset_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("ROI height offset is changed to %d !", msg -> data);
    image_roi_height_offset = msg -> data;
}

void Camera_Focus::roi_resolution_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Resolution ratio is changed to %f !", msg -> data);
    image_roi_resolution = (msg -> data > 1.0)? 1.0 : msg -> data;
    image_roi_resolution = (image_roi_resolution < 0.0)? 0.0 : image_roi_resolution;
}

void Camera_Focus::pub_topic_get()
{
    topic_image_roi_pub = it_roi_pub_.getTopic();
}

void Camera_Focus::pub_init()
{
    ROS_INFO("Publisher %s initiating !", topic_image_sub.c_str());
    it_roi_pub_ = it_roi_ -> advertise(topic_image_roi_pub, queue_size, connect_cb_image_roi, disconnect_cb_image_roi);
}

void Camera_Focus::pub_shutdown()
{
    ROS_WARN("Publisher %s shuting down !", topic_image_roi_pub.c_str());
    it_roi_pub_.shutdown();
}

void Camera_Focus::sub_topic_get()
{   
    topic_image_sub = it_sub_.getTopic();
    topic_image_roi_ratio_sub = roi_ratio_sub_.getTopic();
    topic_image_roi_width_offset_sub = roi_width_offset_sub_.getTopic();
    topic_image_roi_height_offset_sub = roi_height_offset_sub_.getTopic();
    topic_image_roi_resolution_sub = roi_resolution_sub_.getTopic();
}

void Camera_Focus::sub_init()
{
    ROS_INFO("Subscriber %s initiating !", topic_image_sub.c_str());
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &Camera_Focus::image_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_image_roi_ratio_sub.c_str());
    roi_ratio_sub_ = n_.subscribe(topic_image_roi_ratio_sub.c_str(), queue_size, &Camera_Focus::roi_ratio_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_image_roi_width_offset_sub.c_str());
    roi_width_offset_sub_ = n_.subscribe(topic_image_roi_width_offset_sub.c_str(), queue_size, &Camera_Focus::roi_width_offset_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_image_roi_height_offset_sub.c_str());
    roi_height_offset_sub_ = n_.subscribe(topic_image_roi_height_offset_sub.c_str(), queue_size, &Camera_Focus::roi_height_offset_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_image_roi_resolution_sub.c_str());
    roi_resolution_sub_ = n_.subscribe(topic_image_roi_resolution_sub.c_str(), queue_size, &Camera_Focus::roi_resolution_callBack, this);
}

void Camera_Focus::sub_shutdown()
{
    ROS_WARN("Subscriber %s shuting down !", topic_image_sub.c_str());
    it_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_image_roi_ratio_sub.c_str());
    roi_ratio_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_image_roi_width_offset_sub.c_str());
    roi_width_offset_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_image_roi_height_offset_sub.c_str());
    roi_height_offset_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_image_roi_resolution_sub.c_str());
    roi_resolution_sub_.shutdown();
}
#endif
