#pragma once
#ifndef _MOTION_DETECTOR_KNN_H_
#define _MOTION_DETECTOR_KNN_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
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

#include "../../object/MOTION/Motion_KNN.h"
#include "../../ros_comm/config/console_format.h"

using namespace std;
using namespace ros;
using namespace cv;

class Motion_Detector_KNN// : public nodelet::Nodelet
{    
  public:
    string nodeName = "Motion_Detector";
    string topic_image_sub = "Motion_Detector/image";
    string topic_image_motion_pub = "Motion_Detector/image_motion";
    string topic_min_box_length_sub = "Motion_Detector/min_box_length";
    string topic_man_box_length_sub = "Motion_Detector/man_box_length";
    string topic_auto_adjust_sub = "Motion_Detector/auto_adjust";
    string topic_history_sub = "Motion_Detector/history";
    string topic_dist2Threshold_sub = "Motion_Detector/dist2Threshold";
    string topic_detectShadows_sub = "Motion_Detector/detectShadows";
    string topic_learning_rate_sub = "Motion_Detector/learning_rate";
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
    void min_box_length_ratio_callBack(const std_msgs::Float32::ConstPtr& msg);
    void max_box_length_ratio_callBack(const std_msgs::Float32::ConstPtr& msg);
    void auto_adjust_callBack(const std_msgs::Bool::ConstPtr& msg);
    void history_callBack(const std_msgs::Int32::ConstPtr& msg);
    void dist2Threshold_callBack(const std_msgs::Float32::ConstPtr& msg);
    void detectShadows_callBack(const std_msgs::Bool::ConstPtr& msg);
    void learning_rate_callBack(const std_msgs::Float32::ConstPtr& msg);
    void roi_publish();
    
  private:
    MOTION_KNN *motion_knn;
    Mat image;
    Mat image_motion;
    vector<Rect> motion_box;
    bool flag_motion_image = false;
    bool flag_motion_roi = false;
    bool flag_motion_init = false;

  public:
    Motion_Detector_KNN(ros::NodeHandle& nh);
    ~Motion_Detector_KNN();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_motion_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_motion = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_motion    = boost::bind(&Motion_Detector_KNN::connectCb_image_motion, this, _1);
      disconnect_cb_image_motion = boost::bind(&Motion_Detector_KNN::disconnectCb_image_motion, this, _1);
      connect_cb_motion_roi    = boost::bind(&Motion_Detector_KNN::connectCb_motion_roi, this, _1);
      disconnect_cb_motion_roi = boost::bind(&Motion_Detector_KNN::disconnectCb_motion_roi, this, _1);
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
    }
    
};

Motion_Detector_KNN::Motion_Detector_KNN(ros::NodeHandle& nh) : n_(nh)
{    
    motion_knn = new MOTION_KNN;
}

Motion_Detector_KNN::~Motion_Detector_KNN()
{
    delete motion_knn;
}

void Motion_Detector_KNN::run()
{  
    if(!flag_motion_init)
    {
        motion_knn -> init();
        flag_motion_init = true;
    }
    motion_knn -> run(this -> image, this -> image_motion, motion_box);
    //image.copyTo(image_motion);
    /*for(int i = 0; i < motion_box.size(); i++)
    {
       rectangle( image, motion_box[i], cv::Scalar(0, 255, 0), 2, 8, 0 );
    }*/
    roi_publish();
    motion_box.clear();
    image_motion_publish();
}


void Motion_Detector_KNN::image_motion_publish()
{
  if(!flag_motion_image) return;
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_motion).toImageMsg();
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
  it_motion_pub_.publish(msg_image);
}

void Motion_Detector_KNN::image_callBack(const sensor_msgs::ImageConstPtr& msg)
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

void Motion_Detector_KNN::min_box_length_ratio_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Motion min_box_length is changed to %f !", msg -> data);
    motion_knn -> min_box_length_ratio = msg -> data;
}

void Motion_Detector_KNN::max_box_length_ratio_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Motion max_box_length is changed to %f !", msg -> data);
    motion_knn -> max_box_length_ratio = msg -> data;
}

void Motion_Detector_KNN::auto_adjust_callBack(const std_msgs::Bool::ConstPtr& msg)
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
    motion_knn -> auto_adjust = msg -> data;
}

void Motion_Detector_KNN::history_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Motion history is changed to %d !", msg -> data);
    motion_knn -> history = msg -> data;
    flag_motion_init = false;
}

void Motion_Detector_KNN::dist2Threshold_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Motion dist2Threshold is changed to %f !", msg -> data);
    motion_knn -> dist2Threshold = msg -> data;
    flag_motion_init = false;
}

void Motion_Detector_KNN::detectShadows_callBack(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg -> data)
      ROS_INFO("Motion detectShadows on !");
	else
      ROS_INFO("Motion detectShadows off !");
    motion_knn -> detectShadows = msg -> data;
	flag_motion_init = false;
}

void Motion_Detector_KNN::learning_rate_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Motion learning_rate is changed to %f!", msg -> data);
    motion_knn -> learning_rate = msg -> data;
}

void Motion_Detector_KNN::roi_publish()
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
    
void Motion_Detector_KNN::pub_topic_get()
{
    topic_image_motion_pub = it_motion_pub_.getTopic();
    topic_motion_roi_pub = motion_roi_pub_.getTopic();
}

void Motion_Detector_KNN::pub_init()
{
    ROS_INFO("Publisher %s initiating !", topic_image_sub.c_str());
    it_motion_pub_ = it_motion_ -> advertise(topic_image_motion_pub, queue_size, connect_cb_image_motion, disconnect_cb_image_motion);
    ROS_INFO("Publisher %s initiating !", topic_motion_roi_pub.c_str());
    motion_roi_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_motion_roi_pub.c_str(), queue_size, connect_cb_motion_roi, disconnect_cb_motion_roi);
}

void Motion_Detector_KNN::pub_shutdown()
{
    ROS_WARN("Publisher %s shuting down !", topic_image_motion_pub.c_str());
    it_motion_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_motion_roi_pub.c_str());
    motion_roi_pub_.shutdown();
}

void Motion_Detector_KNN::sub_topic_get()
{   
    topic_image_sub = it_sub_.getTopic();
    topic_min_box_length_sub = min_box_length_sub_.getTopic();
    topic_man_box_length_sub = max_box_length_sub_.getTopic();
    topic_auto_adjust_sub = auto_adjust_sub_.getTopic();
    sub_manual_topic_get();
}

void Motion_Detector_KNN::sub_init()
{
    ROS_INFO("Subscriber %s initiating !", topic_image_sub.c_str());
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &Motion_Detector_KNN::image_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_min_box_length_sub.c_str());
    min_box_length_sub_ = n_.subscribe(topic_min_box_length_sub.c_str(), queue_size, &Motion_Detector_KNN::min_box_length_ratio_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_man_box_length_sub.c_str());
    max_box_length_sub_ = n_.subscribe(topic_man_box_length_sub.c_str(), queue_size, &Motion_Detector_KNN::max_box_length_ratio_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_auto_adjust_sub.c_str());
    auto_adjust_sub_ = n_.subscribe(topic_auto_adjust_sub.c_str(), queue_size, &Motion_Detector_KNN::auto_adjust_callBack, this);
    sub_manual_init();
}

void Motion_Detector_KNN::sub_shutdown()
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

void Motion_Detector_KNN::sub_manual_topic_get()
{   
    if(motion_knn -> auto_adjust) return;
    topic_history_sub = history_sub_.getTopic();
    topic_dist2Threshold_sub = dist2Threshold_sub_.getTopic();
    topic_detectShadows_sub = detectShadows_sub_.getTopic();
    topic_learning_rate_sub = learning_rate_sub_.getTopic();
}

void Motion_Detector_KNN::sub_manual_init()
{
    if(motion_knn -> auto_adjust) return;
    ROS_INFO("Subscriber %s initiating !", topic_history_sub.c_str());
    history_sub_ = n_.subscribe(topic_history_sub.c_str(), queue_size, &Motion_Detector_KNN::history_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_dist2Threshold_sub.c_str());
    dist2Threshold_sub_ = n_.subscribe(topic_dist2Threshold_sub.c_str(), queue_size, &Motion_Detector_KNN::dist2Threshold_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_detectShadows_sub.c_str());
    detectShadows_sub_ = n_.subscribe(topic_detectShadows_sub.c_str(), queue_size, &Motion_Detector_KNN::detectShadows_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_learning_rate_sub.c_str());
    learning_rate_sub_ = n_.subscribe(topic_learning_rate_sub.c_str(), queue_size, &Motion_Detector_KNN::learning_rate_callBack, this);
}

void Motion_Detector_KNN::sub_manual_shutdown()
{
    if(motion_knn -> auto_adjust) return;
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

