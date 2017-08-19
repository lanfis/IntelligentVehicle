#pragma once
#ifndef _SUBSCRIBER_CELL_FLOAT32_H_
#define _SUBSCRIBER_CELL_FLOAT32_H_

#include <string>
#include <cstring>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "../config/console_format.h"

using namespace ros;
using namespace std;

class Subscriber_Cell_Float32
{
  private:
    ros::NodeHandle n_;
    ros::Subscriber *sub_;
    string topic_;
    int queue_size_ = 4;
    boost::shared_ptr< std_msgs::Float32 > msg_(new std_msgs::Float32);
	
    void callBack(const std_msgs::Float32::ConstPtr& msg)
    {
      this -> msg_ -> data = msg -> data;
      flag_update = true;
    }
	
  private:
    bool flag_update = false;
	
  public:
    string getTopic() { return topic_; };
	boost::shared_ptr< std_msgs::Float32 > getMsg() { return msg_; };
    bool init(string topic, int queue_size, boost::shared_ptr< std_msgs::Float32 >& msg)
    {
      if(sub_ != NULL) this -> shutdown();
      sub_ = new ros::Subscriber;
      *sub_ = n_.subscribe(topic.c_str(), queue_size, &Subscriber_Cell_Float32::callBack, this);
      this -> topic_ = sub_ -> getTopic();
      this -> queue_size_ = queue_size;
      msg = msg_;
    }
    void shutdown()
    {
      if(sub_ != NULL)
      {
        sub_ -> shutdown();
        delete sub_;
      }
    }
	
  public:
    Subscriber_Cell_Float32(ros::NodeHandle& n);
    ~Subscriber_Cell_Float32();
};

Subscriber_Cell_Float32::Subscriber_Cell_Float32(ros::NodeHandle& n) : n_(n)
{}

Subscriber_Cell_Float32::~Subscriber_Cell_Float32()
{
    shutdown();
}

#endif