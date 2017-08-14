#pragma once
#ifndef _TNN_TRAINER_H_
#define _TNN_TRAINER_H_

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

#include "../../object/TNN/TNN_Train.h"

using namespace std;
using namespace ros;
using namespace cv;

class TNN_Trainer
{    
  public:
    string nodeName = "TNN_Trainer";
    /*string topic_image_sub = "TNN_Trainer/image";
    string topic_image_train_pub = "TNN_Trainer/image_train";
    string topic_label_pub = "TNN_Trainer/label";
    string topic_accuracy_pub = "TNN_Trainer/accuracy";*/
    string topic_train_act_sub = "TNN_Trainer/train_act";
    
  private:
    string ver_ = "1.1";
    int queue_size = 4;
    
    ros::NodeHandle n_;
    /*boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::ImageTransport> it_recognize_;
    boost::shared_ptr<sensor_msgs::Image> msg_image;
    boost::shared_ptr<sensor_msgs::Image> msg_image_train;
    cv_bridge::CvImageConstPtr cv_ptr;
    image_transport::SubscriberStatusCallback connect_cb_image_train;
    image_transport::SubscriberStatusCallback disconnect_cb_image_train;
    ros::SubscriberStatusCallback connect_cb_label;
    ros::SubscriberStatusCallback disconnect_cb_label;*/
      
    /*image_transport::Subscriber it_sub_;
    image_transport::Publisher it_recognize_pub_;
    ros::Publisher label_pub_;
    ros::Publisher accuracy_pub_;*/
    ros::Subscriber train_act_sub_;
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    /*void connectCb_image_train(const image_transport::SingleSubscriberPublisher& ssp)
    {
      if(it_recognize_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_image_train_pub.c_str());
      flag_image_train = true;
      if(!(flag_image_train & flag_label))
      {
        sub_init();
      }
    }
    void disconnectCb_image_train(const image_transport::SingleSubscriberPublisher&)
    {
      if(it_recognize_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_image_train_pub.c_str());
      flag_image_train = false;
      if(!(flag_image_train | flag_label))
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
      if(!(flag_image_train & flag_label))
      {
        sub_init();
      }
    }
    void disconnectCb_label(const ros::SingleSubscriberPublisher&)
    {
      if(accuracy_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_label_pub.c_str());
      flag_label = false;
      if(!(flag_image_train | flag_label))
      {
        flag_motion_init = false;
        sub_shutdown();
      }
    }*/
    
  private:
    /*void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void image_train_publish();
    void accuracy_publish();
    void label_publish();*/
    void train_act_callBack(const std_msgs::Bool::ConstPtr& msg);
    
  private:
    TNN_Train *tnn_train;
    /*Mat image;
    Mat image_train;
    bool flag_image_train = false;
    bool flag_label = false;
    bool flag_accuracy = false;*/
    bool flag_train_init = false;
    bool flag_train_act = false;
    

  public:
    TNN_Trainer(ros::NodeHandle& nh);
    ~TNN_Trainer();
    void run();
    void train_init();
    void mnist_load();
    
  public:
    virtual void init()
    {
      /*it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      it_recognize_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(n_));
      msg_image = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      msg_image_train = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
      connect_cb_image_train    = boost::bind(&TNN_Trainer::connectCb_image_train, this, _1);
      disconnect_cb_image_train = boost::bind(&TNN_Trainer::disconnectCb_image_train, this, _1);
      connect_cb_label    = boost::bind(&TNN_Trainer::connectCb_label, this, _1);
      disconnect_cb_label = boost::bind(&TNN_Trainer::disconnectCb_label, this, _1);*/
      pub_init();
      pub_topic_get();
      sub_init();
    }
    
};

TNN_Trainer::TNN_Trainer(ros::NodeHandle& nh) : n_(nh)
{    
    tnn_train = new TNN_Train;
}

TNN_Trainer::~TNN_Trainer()
{
    delete tnn_train;
}

void TNN_Trainer::run()
{  
    if(!flag_train_act) return;
    train_init();
    tnn_train -> run();
    flag_train_act = false;
}

void TNN_Trainer::train_init()
{
    if(flag_train_init) return;
    ROS_INFO("TNN_Trainer initializing ...");
    if(tnn_train != NULL)
    {
        delete tnn_train;
        tnn_train = new TNN_Train;
    }
    ROS_INFO("-- data path : %s", tnn_train -> data_dir_path.c_str());
    ROS_INFO("-- defalut weight path : %s",tnn_train -> tnn_fileName.c_str());
    ROS_INFO("TNN_Trainer constructing network ...");
    if(!tnn_train -> construct_net())
    {
        ROS_WARN("TNN_Trainer construct network fail !");
    }
    ROS_INFO("TNN_Trainer weight loading ...");
    if(tnn_train -> weight_load())
    {
      ROS_INFO("TNN_Trainer weight loading successfully !");
    }
    else
    {
      ROS_WARN("TNN_Trainer weight loading successfully !");
    }
    ROS_INFO("TNN_Trainer initializing done !");
}

void TNN_Trainer::mnist_load()
{
  cout << "-- mnist train data loading with default working space path ...";
  if(tnn_train -> mnist_train_load())
  {
    cout << "ok !\n";
  }
  else
  {
    cout << "fail !\n";
  }
  cout << "-- mnist test data loading with default working space path ...";
  if(tnn_train -> mnist_test_load())
  {
    cout << "ok !\n";
  }
  else
  {
    cout << "fail !\n";
  }
}
/*
void TNN_Trainer::image_train_publish()
{
  if(!flag_image_train) return;
  msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_train).toImageMsg();
  it_recognize_pub_.publish(msg_image);
}

void TNN_Trainer::image_callBack(const sensor_msgs::ImageConstPtr& msg)
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

void TNN_Trainer::label_publish()
{
    if(!flag_label) return;
    std_msgs:: msg;
}
*/
void TNN_Trainer::train_act_callBack(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("TNN_Trainer training start !");
    flag_train_act = msg -> data;
}

void TNN_Trainer::pub_topic_get()
{
    //topic_image_train_pub = it_recognize_pub_.getTopic();
    //topic_accuracy_pub = accuracy_pub_.getTopic();
}

void TNN_Trainer::pub_init()
{
    /*ROS_INFO("Publisher %s initiating !", topic_image_sub.c_str());
    it_recognize_pub_ = it_recognize_ -> advertise(topic_image_train_pub, queue_size, connect_cb_image_train, disconnect_cb_image_train);
    ROS_INFO("Publisher %s initiating !", topic_accuracy_pub.c_str());
    accuracy_pub_ = n_.advertise< sensor_msgs::RegionOfInterest>(topic_accuracy_pub.c_str(), queue_size, connect_cb_label, disconnect_cb_label);*/
}

void TNN_Trainer::pub_shutdown()
{
    /*ROS_WARN("Publisher %s shuting down !", topic_image_train_pub.c_str());
    it_recognize_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_accuracy_pub.c_str());
    accuracy_pub_.shutdown();*/
}

void TNN_Trainer::sub_topic_get()
{   
    /*topic_image_sub = it_sub_.getTopic();
    topic_min_box_length_sub = min_box_length_sub_.getTopic();
    topic_man_box_length_sub = max_box_length_sub_.getTopic();
    topic_auto_adjust_sub = auto_adjust_sub_.getTopic();*/
    topic_train_act_sub = train_act_sub_.getTopic();
}

void TNN_Trainer::sub_init()
{
    /*ROS_INFO("Subscriber %s initiating !", topic_image_sub.c_str());
    it_sub_ = it_ -> subscribe(topic_image_sub.c_str(), queue_size, &TNN_Trainer::image_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_min_box_length_sub.c_str());
    min_box_length_sub_ = n_.subscribe(topic_min_box_length_sub.c_str(), queue_size, &TNN_Trainer::min_box_length_ratio_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_man_box_length_sub.c_str());
    max_box_length_sub_ = n_.subscribe(topic_man_box_length_sub.c_str(), queue_size, &TNN_Trainer::max_box_length_ratio_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_auto_adjust_sub.c_str());
    auto_adjust_sub_ = n_.subscribe(topic_auto_adjust_sub.c_str(), queue_size, &TNN_Trainer::auto_adjust_callBack, this);*/
    ROS_INFO("Subscriber %s initiating !", topic_train_act_sub.c_str());
    train_act_sub_ = n_.subscribe(topic_train_act_sub.c_str(), queue_size, &TNN_Trainer::train_act_callBack, this);
}

void TNN_Trainer::sub_shutdown()
{
    /*ROS_WARN("Subscriber %s shuting down !", topic_image_sub.c_str());
    it_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_min_box_length_sub.c_str());
    min_box_length_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_man_box_length_sub.c_str());
    max_box_length_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_auto_adjust_sub.c_str());
    auto_adjust_sub_.shutdown();*/
    train_act_sub_.shutdown();
}

#endif

