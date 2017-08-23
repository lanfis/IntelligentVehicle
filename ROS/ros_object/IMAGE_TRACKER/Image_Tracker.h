#pragma once
#ifndef _IMAGE_TRACKER_H_
#define _IMAGE_TRACKER_H_

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

#include "../../object/KCF/KCF.h"

using namespace std;
using namespace ros;
using namespace cv;

class Image_Tracker
{    
  public:
    string nodeName = "Image_Tracker";
    string topic_image_sub = "Image_Tracker/image";
    string topic_image_tracker_pub = "Image_Tracker/image_tracker";
    string topic_tracker_track_pub = "Image_Tracker/tracker_track";
    string topic_tracker_roi_sub = "Image_Tracker/tracker_roi";
    string topic_auto_adjust_sub = "Image_Tracker/auto_adjust";
     
    string topic_interp_factor_sub = "Image_Tracker/interp_factor";// linear interpolation factor for adaptation
    string topic_sigma_sub = "Image_Tracker/sigma";// gaussian kernel bandwidth
    string topic_lambda_sub = "Image_Tracker/lambda";// regularization
    string topic_cell_size_sub = "Image_Tracker/cell_size";// HOG cell size
    string topic_cell_sizeQ_sub = "Image_Tracker/cell_sizeQ";// cell size^2, to avoid repeated operations
    string topic_padding_sub = "Image_Tracker/padding";// extra area surrounding the target
    string topic_output_sigma_factor_sub = "Image_Tracker/output_sigma_factor";// bandwidth of gaussian target
    string topic_template_size_sub = "Image_Tracker/template_size";// template size
    string topic_scale_step_sub = "Image_Tracker/scale_step";// scale step for multi-scale estimation
    string topic_scale_weight_sub = "Image_Tracker/scale_weight";// to downweight detection scores of other scales for added stability
    
  private:
    string ver_ = "2.1";
    int queue_size = 4;
    
    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_tracker_;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image;
    boost::shared_ptr<sensor_msgs::Image>/*sensor_msgs::ImagePtr*/ msg_image_tracker;
    cv_bridge::CvImageConstPtr/*CvImagePtr*/ cv_ptr;
    image_transport::SubscriberStatusCallback connect_cb_image_tracker;
    image_transport::SubscriberStatusCallback disconnect_cb_image_tracker;
    ros::SubscriberStatusCallback connect_cb_tracker_track;
    ros::SubscriberStatusCallback disconnect_cb_tracker_track;
      
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_tracker_pub_;
    ros::Publisher tracker_track_pub_;
    ros::Subscriber tracker_roi_sub_;
    ros::Subscriber auto_adjust_sub_;
    
    ros::Subscriber interp_factor_sub_;
    ros::Subscriber sigma_sub_;
    ros::Subscriber lambda_sub_;
    ros::Subscriber cell_size_sub_;
    ros::Subscriber cell_sizeQ_sub_;
    ros::Subscriber padding_sub_;
    ros::Subscriber output_sigma_factor_sub_;
    ros::Subscriber template_size_sub_;
    ros::Subscriber scale_step_sub_;
    ros::Subscriber scale_weight_sub_;
    
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
    void connectCb_image_tracker(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_tracker_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_tracker_pub.c_str());
      flag_kcft_image = true;
      if(!(flag_kcft_image & flag_kcft_track))
      {
        sub_init();
      }
    }
    void disconnectCb_image_tracker(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_tracker_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_tracker_pub.c_str());
      flag_kcft_image = false;
      if(!(flag_kcft_image | flag_kcft_track))
      {
        flag_kcft_init = false;
        sub_shutdown();
      }
    }
    void connectCb_tracker_track(const ros::SingleSubscriberPublisher& ssp)
    {
      if(tracker_track_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_tracker_track_pub.c_str());
      flag_kcft_track = true;
      if(!(flag_kcft_image & flag_kcft_track))
      {
        sub_init();
      }
    }
    void disconnectCb_tracker_track(const ros::SingleSubscriberPublisher&)
    {
      if(tracker_track_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_tracker_track_pub.c_str());
      flag_kcft_track = false;
      if(!(flag_kcft_image | flag_kcft_track))
      {
        flag_kcft_init = false;
        sub_shutdown();
      }
    }
    
  private:
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_tracker_publish();
    void kcft_track_publish();
    void tracker_roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg);
    void auto_adjust_callBack(const std_msgs::Bool::ConstPtr& msg);
    void interp_factor_callBack(const std_msgs::Float32::ConstPtr& msg);
    void sigma_callBack(const std_msgs::Float32::ConstPtr& msg);
    void lambda_callBack(const std_msgs::Float32::ConstPtr& msg);
    void cell_size_callBack(const std_msgs::Int32::ConstPtr& msg);
    void cell_sizeQ_callBack(const std_msgs::Int32::ConstPtr& msg);
    void padding_callBack(const std_msgs::Float32::ConstPtr& msg);
    void output_sigma_factor_callBack(const std_msgs::Float32::ConstPtr& msg);
    void template_size_callBack(const std_msgs::Int32::ConstPtr& msg);
    void scale_step_callBack(const std_msgs::Float32::ConstPtr& msg);
    void scale_weight_callBack(const std_msgs::Float32::ConstPtr& msg);
    void roi_publish();
    
  private:
    KCF *kcft;
    Mat image;
    Mat image_kcft;
    Rect kcft_track;
    Rect kcft_roi;
    bool flag_kcft_image = false;
    bool flag_kcft_image_sub = false;
    bool flag_kcft_track = false;
    bool flag_kcft_init = false;
    bool flag_auto_adjust = false;

  public:
    Image_Tracker(ros::NodeHandle& nh);
    ~Image_Tracker();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_tracker_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_tracker = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_tracker    = boost::bind(&Image_Tracker::connectCb_image_tracker, this, _1);
      disconnect_cb_image_tracker = boost::bind(&Image_Tracker::disconnectCb_image_tracker, this, _1);
      connect_cb_tracker_track    = boost::bind(&Image_Tracker::connectCb_tracker_track, this, _1);
      disconnect_cb_tracker_track = boost::bind(&Image_Tracker::disconnectCb_tracker_track, this, _1);
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
    }
    
};

Image_Tracker::Image_Tracker(ros::NodeHandle& nh) : n_(nh)
{    
    kcft = new KCF;
    kcft_roi.x = 0;
    kcft_roi.y = 0;
    kcft_roi.width = 1;
    kcft_roi.height = 1;
}

Image_Tracker::~Image_Tracker()
{
    delete kcft;
}

void Image_Tracker::run()
{  
    if(!flag_kcft_image_sub) return;
    if(!flag_kcft_init)
    {
        kcft -> init(this -> image, kcft_roi);
        flag_kcft_init = true;
    }
    kcft -> run(this -> image, kcft_track);
    image_tracker_publish();
    kcft_track_publish();
    flag_kcft_image_sub = false;
}

void Image_Tracker::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      //cv_ptr -> image.copyTo(*image);
      this -> image = cv_ptr -> image;
      flag_kcft_image_sub = true;
      run();
      return;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

void Image_Tracker::image_tracker_publish()
{
  if(!flag_kcft_image) return;
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
  it_tracker_pub_.publish(msg_image);
}

void Image_Tracker::kcft_track_publish()
{
    if(!flag_kcft_track) return;
    sensor_msgs::RegionOfInterest msg;
    msg.x_offset = kcft_track.x + kcft_track.width/2;      
    msg.y_offset = kcft_track.y + kcft_track.height/2;      
    msg.width    = kcft_track.width;      
    msg.height   = kcft_track.height;     
    tracker_track_pub_.publish(msg);
}

void Image_Tracker::tracker_roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg)
{
    ROS_INFO("Image_Tracker kcft_roi is changed to x= %d, y= %d, width= %d, height= %d !", msg -> x_offset, msg -> y_offset, msg -> width, msg -> height);
    kcft_roi.x = (msg -> x_offset - msg -> width/2 < 0 )? 0 : msg -> x_offset - msg -> width/2;
    kcft_roi.y = (msg -> y_offset - msg -> height/2 < 0)? 0 : msg -> y_offset - msg -> height/2;
    kcft_roi.width =  (msg -> x_offset + msg -> width/2 > image.cols )? image.cols - kcft_roi.x : msg -> x_offset + msg -> width/2  - kcft_roi.x;
    kcft_roi.height = (msg -> y_offset + msg -> height/2 > image.rows)? image.rows - kcft_roi.y : msg -> y_offset + msg -> height/2 - kcft_roi.y;
    flag_kcft_init = false;
}
void Image_Tracker::auto_adjust_callBack(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg -> data)
    {
      ROS_INFO("Image_Tracker auto_adjust on !");
      sub_manual_init();
      sub_manual_topic_get();
    }
    else
    {
      ROS_INFO("Image_Tracker auto_adjust off !");
      sub_manual_shutdown();
    }
    flag_auto_adjust = msg -> data;
}

void Image_Tracker::interp_factor_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Image_Tracker interp_factor is changed to %f !", msg -> data);
    kcft -> interp_factor = msg -> data;
    kcft_roi = kcft_track;
    flag_kcft_init = false;
}

void Image_Tracker::sigma_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Image_Tracker sigma is changed to %f !", msg -> data);
    kcft -> sigma = msg -> data;
    kcft_roi = kcft_track;
    flag_kcft_init = false;
}

void Image_Tracker::lambda_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Image_Tracker lambda is changed to %f !", msg -> data);
    kcft -> lambda = msg -> data;
    kcft_roi = kcft_track;
    flag_kcft_init = false;
}

void Image_Tracker::cell_size_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Image_Tracker cell_size is changed to %d !", msg -> data);
    kcft -> cell_size = msg -> data;
    kcft_roi = kcft_track;
    flag_kcft_init = false;
}

void Image_Tracker::cell_sizeQ_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Image_Tracker cell_sizeQ is changed to %d !", msg -> data);
    kcft -> cell_sizeQ = msg -> data;
    kcft_roi = kcft_track;
    flag_kcft_init = false;
}

void Image_Tracker::padding_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Image_Tracker padding is changed to %f !", msg -> data);
    kcft -> padding = msg -> data;
    kcft_roi = kcft_track;
    flag_kcft_init = false;
}

void Image_Tracker::output_sigma_factor_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Image_Tracker output_sigma_factor is changed to %f !", msg -> data);
    kcft -> output_sigma_factor = msg -> data;
    kcft_roi = kcft_track;
    flag_kcft_init = false;
}

void Image_Tracker::template_size_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Image_Tracker template_size is changed to %d !", msg -> data);
    kcft -> template_size = msg -> data;
    kcft_roi = kcft_track;
    flag_kcft_init = false;
}

void Image_Tracker::scale_step_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Image_Tracker scale_step is changed to %f !", msg -> data);
    kcft -> scale_step = msg -> data;
    kcft_roi = kcft_track;
    flag_kcft_init = false;
}

void Image_Tracker::scale_weight_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Image_Tracker scale_weight is changed to %f !", msg -> data);
    kcft -> scale_weight = msg -> data;
    kcft_roi = kcft_track;
    flag_kcft_init = false;
}

void Image_Tracker::pub_topic_get()
{
    topic_image_tracker_pub = it_tracker_pub_.getTopic();
    topic_tracker_track_pub = tracker_track_pub_.getTopic();
}

void Image_Tracker::pub_init()
{
    ROS_INFO("Publisher %s initiating !", topic_image_tracker_pub.c_str());
    it_tracker_pub_ = it_tracker_ -> advertise(topic_image_tracker_pub, queue_size, connect_cb_image_tracker, disconnect_cb_image_tracker);
    ROS_INFO("Publisher %s initiating !", topic_tracker_track_pub.c_str());
    tracker_track_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_tracker_track_pub.c_str(), queue_size, connect_cb_tracker_track, disconnect_cb_tracker_track);
}

void Image_Tracker::pub_shutdown()
{
    ROS_WARN("Publisher %s shuting down !", topic_image_tracker_pub.c_str());
    it_tracker_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_tracker_track_pub.c_str());
    tracker_track_pub_.shutdown();
}

void Image_Tracker::sub_topic_get()
{   
    topic_image_sub = it_sub_.getTopic();
    topic_tracker_roi_sub = tracker_roi_sub_.getTopic();
    topic_auto_adjust_sub = auto_adjust_sub_.getTopic();
    sub_manual_topic_get();
}

void Image_Tracker::sub_init()
{
    ROS_INFO("Subscriber %s initiating !", topic_image_sub.c_str());
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &Image_Tracker::image_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_tracker_roi_sub.c_str());
    tracker_roi_sub_ = n_.subscribe(topic_tracker_roi_sub.c_str(), queue_size, &Image_Tracker::tracker_roi_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_auto_adjust_sub.c_str());
    auto_adjust_sub_ = n_.subscribe(topic_auto_adjust_sub.c_str(), queue_size, &Image_Tracker::auto_adjust_callBack, this);
    sub_manual_init();
}

void Image_Tracker::sub_shutdown()
{
    ROS_WARN("Subscriber %s shuting down !", topic_image_sub.c_str());
    it_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_tracker_roi_sub.c_str());
    tracker_roi_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_auto_adjust_sub.c_str());
    auto_adjust_sub_.shutdown();
    sub_manual_shutdown();
}

void Image_Tracker::sub_manual_topic_get()
{   
    if(flag_auto_adjust) return;
    topic_interp_factor_sub = interp_factor_sub_.getTopic();
    topic_sigma_sub = sigma_sub_.getTopic();
    topic_lambda_sub = lambda_sub_.getTopic();
    topic_cell_size_sub = cell_size_sub_.getTopic();
    topic_cell_sizeQ_sub = cell_sizeQ_sub_.getTopic();
    topic_padding_sub = padding_sub_.getTopic();
    topic_output_sigma_factor_sub = output_sigma_factor_sub_.getTopic();
    topic_template_size_sub = template_size_sub_.getTopic();
    topic_scale_step_sub = scale_step_sub_.getTopic();
    topic_scale_weight_sub = scale_weight_sub_.getTopic();
}

void Image_Tracker::sub_manual_init()
{
    if(flag_auto_adjust) return;
    ROS_INFO("Subscriber %s initiating !", topic_interp_factor_sub.c_str());
    interp_factor_sub_ = n_.subscribe(topic_interp_factor_sub.c_str(), queue_size, &Image_Tracker::interp_factor_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_sigma_sub.c_str());
    sigma_sub_ = n_.subscribe(topic_sigma_sub.c_str(), queue_size, &Image_Tracker::sigma_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_lambda_sub.c_str());
    lambda_sub_ = n_.subscribe(topic_lambda_sub.c_str(), queue_size, &Image_Tracker::lambda_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_cell_size_sub.c_str());
    cell_size_sub_ = n_.subscribe(topic_cell_size_sub.c_str(), queue_size, &Image_Tracker::cell_size_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_cell_sizeQ_sub.c_str());
    cell_sizeQ_sub_ = n_.subscribe(topic_cell_sizeQ_sub.c_str(), queue_size, &Image_Tracker::cell_sizeQ_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_padding_sub.c_str());
    padding_sub_ = n_.subscribe(topic_padding_sub.c_str(), queue_size, &Image_Tracker::padding_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_output_sigma_factor_sub.c_str());
    output_sigma_factor_sub_ = n_.subscribe(topic_output_sigma_factor_sub.c_str(), queue_size, &Image_Tracker::output_sigma_factor_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_template_size_sub.c_str());
    template_size_sub_ = n_.subscribe(topic_template_size_sub.c_str(), queue_size, &Image_Tracker::template_size_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_scale_step_sub.c_str());
    scale_step_sub_ = n_.subscribe(topic_scale_step_sub.c_str(), queue_size, &Image_Tracker::scale_step_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_scale_weight_sub.c_str());
    scale_weight_sub_ = n_.subscribe(topic_scale_weight_sub.c_str(), queue_size, &Image_Tracker::scale_weight_callBack, this);
}

void Image_Tracker::sub_manual_shutdown()
{
    if(flag_auto_adjust) return;
    ROS_WARN("Subscriber %s shuting down !", topic_interp_factor_sub.c_str());
    interp_factor_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_sigma_sub.c_str());
    sigma_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_lambda_sub.c_str());
    lambda_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_cell_size_sub.c_str());
    cell_size_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_cell_sizeQ_sub.c_str());
    cell_sizeQ_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_padding_sub.c_str());
    padding_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_output_sigma_factor_sub.c_str());
    output_sigma_factor_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_template_size_sub.c_str());
    template_size_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_scale_step_sub.c_str());
    scale_step_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_scale_weight_sub.c_str());
    scale_weight_sub_.shutdown();
}
#endif

