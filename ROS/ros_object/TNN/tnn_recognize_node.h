#pragma once
#ifndef _TNN_RECOGNIZE_NODE_H_
#define _TNN_RECOGNIZE_NODE_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <string>
#include <cstring>
#include <sstream>

#include "TNN/TNN_Recognize.h"



using namespace ros;
using namespace std;



class TNN_Recognize_Node
{
  public:
    string nodeName;
    string topic_image_sub;
    string topic_label_pub;
    string topic_accuracy_pub;
    string topic_activation_sub;
    string topic_status_pub;

  private:
    float ver_ = 1.0;
    int queue_size;
    bool flag_activation;
    bool flag_image_update;
    bool flag_label_update;
    bool flag_accuracy_update;
    
    TNN_Recognize *tnn_recognize;
    
  private:
    ros::AsyncSpinner spinner;
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber tnn_recognize_node_image_sub_;
    ros::Publisher tnn_recognize_node_label_pub_;
    ros::Publisher tnn_recognize_node_accuracy_pub_;
    ros::Subscriber tnn_recognize_node_activation_sub_;
    ros::Publisher tnn_recognize_node_status_pub_;

    cv_bridge::CvImagePtr cv_ptr;
    Mat img;
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
    void label_publish();
    void accuracy_publish();
    void status_publish(string status);
    void activation_callBack(const std_msgs::Bool::ConstPtr& msg);

  public:   
    bool activation;
/*
    bool update_image;
    bool update_label;
    bool update_accuracy;
*/

  public:
    TNN_Recognize_Node(int thread);
    ~TNN_Recognize_Node();
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void run();
};

TNN_Recognize_Node::TNN_Recognize_Node(int thread) : it_(n_), spinner(thread)
{ 
  nodeName = "TNN_Recognize_Node";
  topic_image_sub = nodeName+"/image";
  topic_label_pub = nodeName+"/label";
  topic_accuracy_pub = nodeName+"/accuracy";
  topic_activation_sub = nodeName+"/activation";
  topic_status_pub = nodeName+"/status";
  
  activation = true;
  flag_activation = true;
//  update_image = false;
  flag_image_update = false;
//  update_label = false;
  flag_label_update = false;
//  update_accuracy = false;
  flag_accuracy_update = false;
  
  pub_init();
  pub_topic_get();
  sub_init();
  sub_topic_get();
  status_publish(nodeName + " initializing ...");
  cout << "-- initializing tiny neural network\n";
  tnn_recognize = new TNN_Recognize;
  cout << "-- constructing ...\n";
  tnn_recognize -> construct_net();
//  cout << "-- setting working directory path ...\n";
//  tnn_recognize -> working_directory_set("/home/adel/Dropbox/ROS/ros_package/TNN/TNN_WORK_SPACE");
  cout << "-- weight loading with default working space path ...";
  if(tnn_recognize -> weight_load())
    cout << "ok !\n";
  else
    cout << "fail !\n";
  cout << "-- defalut weight path : " << tnn_recognize -> tnn_fileName << endl;
  
  spinner.start();
  cout << "-- recognizing ...\n";
  run();
}

TNN_Recognize_Node::~TNN_Recognize_Node()
{
    delete tnn_recognize;
}

void TNN_Recognize_Node::run()
{
    if(!activation)
    {
        if(flag_activation)
        {
//          pub_shutdown();
//          sub_shutdown();
          flag_activation = false;
        }
        return;
    }
    if(!flag_activation)
    {
      pub_init();
      sub_init();
      flag_activation = true;
    }

    if(flag_image_update)
    {
      if(tnn_recognize -> run(img))
      {
        label_publish();
        accuracy_publish();
      }
    }
    //activation = false;
    flag_image_update = false;
}

void TNN_Recognize_Node::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    if(!flag_activation) return;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //cv_ptr -> image.copyTo(img);
    img = cv_ptr -> image;
    flag_image_update = true;
}

void TNN_Recognize_Node::label_publish()
{
    std_msgs::Int32 msg;
    msg.data = tnn_recognize -> label;
    tnn_recognize_node_label_pub_.publish(msg);
}

void TNN_Recognize_Node::accuracy_publish()
{
    std_msgs::Float32 msg;
    msg.data = tnn_recognize -> accuracy;
    tnn_recognize_node_accuracy_pub_.publish(msg);
}

void TNN_Recognize_Node::activation_callBack(const std_msgs::Bool::ConstPtr& msg)
{
    activation = msg -> data;
    if(activation)
    {
//      ROS_WARN("%s activated !", nodeName.c_str());
      status_publish(nodeName + " activated !");
    }
    else
    {
//      ROS_WARN("%s inactivated !", nodeName.c_str());
      status_publish(nodeName + " inactivated !");
    }
    this -> run();
}

void TNN_Recognize_Node::status_publish(string status)
{
    std_msgs::String msg;
    msg.data = status.c_str();
    tnn_recognize_node_status_pub_.publish(msg);
}

void TNN_Recognize_Node::pub_topic_get()
{
    topic_label_pub = tnn_recognize_node_label_pub_.getTopic();
    topic_accuracy_pub = tnn_recognize_node_accuracy_pub_.getTopic();
    topic_status_pub = tnn_recognize_node_status_pub_.getTopic();
}

void TNN_Recognize_Node::pub_init()
{
  tnn_recognize_node_label_pub_ = n_.advertise< std_msgs::Int32>(topic_label_pub.c_str(), queue_size);
  tnn_recognize_node_accuracy_pub_ = n_.advertise< std_msgs::Float32>(topic_accuracy_pub.c_str(), queue_size);
  tnn_recognize_node_status_pub_ = n_.advertise< std_msgs::String>(topic_status_pub.c_str(), queue_size);
}

void TNN_Recognize_Node::pub_shutdown()
{
  tnn_recognize_node_label_pub_.shutdown();
  tnn_recognize_node_accuracy_pub_.shutdown();
}

void TNN_Recognize_Node::sub_topic_get()
{
    topic_image_sub = tnn_recognize_node_image_sub_.getTopic();
    topic_activation_sub = tnn_recognize_node_activation_sub_.getTopic();
}

void TNN_Recognize_Node::sub_init()
{
  tnn_recognize_node_image_sub_ = it_.subscribe(topic_image_sub.c_str(), queue_size, &TNN_Recognize_Node::image_callBack, this);
  tnn_recognize_node_activation_sub_ = n_.subscribe(topic_activation_sub.c_str(), queue_size, &TNN_Recognize_Node::activation_callBack, this);
}

void TNN_Recognize_Node::sub_shutdown()
{
  tnn_recognize_node_image_sub_.shutdown();
}
#endif
