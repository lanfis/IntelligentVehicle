#pragma once
#ifndef _PUBLISHER_CELL_STRING_H_
#define _PUBLISHER_CELL_STRING_H_

#include <string>
#include <cstring>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "../config/console_format.h"

using namespace ros;
using namespace std;

class Publisher_Cell_String
{
  private:
    ros::NodeHandle n_;
    ros::Publisher *pub_;
    string topic_;
    int queue_size_ = 4;
    boost::shared_ptr< std_msgs::String > msg_(new std_msgs::String);
	
  private:
    bool flag_update = false;
	
  public:
    string getTopic() { return topic_; };
	boost::shared_ptr< std_msgs::String > getMsg() { return msg_; };
    bool init(string topic, int queue_size)
    {
      if(pub_ != NULL) this -> shutdown();
      pub_ = new ros::Publisher;
      *pub_ = n_.advertise< std_msgs::String >(topic.c_str(), queue_size);
      this -> topic_ = topic;
      this -> queue_size_ = queue_size;
    }
    bool init(string topic, int queue_size, &ros::SubscriberStatusCallback connect_cb, &ros::SubscriberStatusCallback disconnect_cb)
    {
      if(pub_ != NULL) this -> shutdown();
      pub_ = new ros::Publisher;
      *pub_ = n_.advertise< std_msgs::String >(topic.c_str(), queue_size, connect_cb, disconnect_cb);
      this -> topic_ = pub_ -> getTopic();
      this -> queue_size_ = queue_size;      
    }
    void publish(boost::shared_ptr< std_msgs::String > &msg)
    {
      if(pub_.getNumSubscribers() == 0) return;
      msg_ -> data = msg -> data;
      pub_ -> publish(msg_);
    }
    void publish(std_msgs::String &msg)
    {
      if(pub_.getNumSubscribers() == 0) return;
      msg_ -> data = msg.data;
      pub_ -> publish(msg_);
    }
    void publish(String &msg)
    {
      if(pub_.getNumSubscribers() == 0) return;
      msg_ -> data = msg;
      pub_ -> publish(msg_);
    }
    void shutdown()
    {
      if(pub_ != NULL)
      {
        pub_ -> shutdown();
        delete pub_;
      }
    }
	
  public:
    Publisher_Cell_String(ros::NodeHandle& n);
    ~Publisher_Cell_String();
};

Publisher_Cell_String::Publisher_Cell_String(ros::NodeHandle& n) : n_(n)
{}

Publisher_Cell_String::~Publisher_Cell_String()
{
    shutdown();
}

#endif
