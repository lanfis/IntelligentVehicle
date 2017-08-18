#pragma once
#ifndef _TNN_TRAIN_NODE_H_
#define _TNN_TRAIN_NODE_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <string>
#include <cstring>
#include <sstream>

#include "TNN/TNN_Train.h"



using namespace ros;
using namespace std;



class TNN_Train_Node
{
  public:
    string nodeName;
    string topic_activation_sub;
    string topic_status_pub;

  private:
    float ver_ = 1.0;
    int queue_size;
    bool flag_activation;
    
    TNN_Train *tnn_train;
    
  private:
    ros::AsyncSpinner spinner;
    ros::NodeHandle n_;
    ros::Subscriber tnn_train_node_activation_sub_;
    ros::Publisher tnn_train_node_status_pub_;

    cv_bridge::CvImagePtr cv_ptr;
    void status_publish(string status);
    void activation_callBack(const std_msgs::Bool::ConstPtr& msg);

  public:   
    bool activation;

  public:
    TNN_Train_Node(int thread);
    ~TNN_Train_Node();
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void run();
};

TNN_Train_Node::TNN_Train_Node(int thread) : spinner(thread)
{ 
  nodeName = "TNN_Train_Node";
  topic_activation_sub = nodeName+"/activation";
  topic_status_pub = nodeName+"/status";
  
  activation = true;
  flag_activation = true;
  
  pub_init();
  pub_topic_get();
  sub_init();
  sub_topic_get();
  status_publish(nodeName + " initializing ...");
  cout << "-- initializing tiny neural network\n";
  tnn_train = new TNN_Train;
  cout << "-- constructing ...\n";
  tnn_train -> construct_net();
//  cout << "-- setting working directory path ...\n";
//  tnn_train -> working_directory_set("/home/adel/Dropbox/ROS/ros_package/TNN/TNN_WORK_SPACE");
  cout << "-- data path : " << tnn_train -> data_dir_path << endl;
  cout << "-- weight loading with default working space path ...";
  if(tnn_train -> weight_load())
  {
    cout << "ok !\n";
  }
  else
  {
    cout << "fail !\n";
  }
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
  
  spinner.start();
  cout << "-- training ...\n";
  run();
}

TNN_Train_Node::~TNN_Train_Node()
{
    delete tnn_train;
}

void TNN_Train_Node::run()
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
    
    tnn_train -> run();
    activation = false;
}


void TNN_Train_Node::activation_callBack(const std_msgs::Bool::ConstPtr& msg)
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

void TNN_Train_Node::status_publish(string status)
{
    std_msgs::String msg;
    msg.data = status.c_str();
    tnn_train_node_status_pub_.publish(msg);
}

void TNN_Train_Node::pub_topic_get()
{
    topic_status_pub = tnn_train_node_status_pub_.getTopic();
}

void TNN_Train_Node::pub_init()
{
  tnn_train_node_status_pub_ = n_.advertise< std_msgs::String>(topic_status_pub.c_str(), queue_size);
}

void TNN_Train_Node::pub_shutdown()
{
}

void TNN_Train_Node::sub_topic_get()
{
    topic_activation_sub = tnn_train_node_activation_sub_.getTopic();
}

void TNN_Train_Node::sub_init()
{
  tnn_train_node_activation_sub_ = n_.subscribe(topic_activation_sub.c_str(), queue_size, &TNN_Train_Node::activation_callBack, this);
}

void TNN_Train_Node::sub_shutdown()
{
}
#endif
