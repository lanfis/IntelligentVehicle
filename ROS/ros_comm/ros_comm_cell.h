#pragma once
#ifndef _ROS_COMM_CELL_H_
#define _ROS_COMM_CELL_H_

#include <string>
#include <cstring>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

using namespace ros;
using namespace std;

class ROS_Comm_Cell
{
  private:
	ros::NodeHandle n_;
    ros::Publisher  *pub_;
    ros::Subscriber  *sub_;
    //image_transport::ImageTransport it_;
	bool flag_activation;
	
    void callBack_string(const std_msgs::String::ConstPtr& msg)
    {
      this -> msg_string -> data = msg -> data;
      flag_update = true;
    }
	
  public:
	bool flag_update;
	string topic;
    int queue_size;
	int num_link;
	std_msgs::String *msg_string;
	
  public:
	ROS_Comm_Cell(ros::NodeHandle& n) : n_(n), queue_size(4), flag_activation(false) { linkCell = this; };
	~ROS_Comm_Cell();
    bool pub(string topic, std_msgs::String *&msg)
    {
      if(!flag_activation)
	  {
		pub_ = new ros::Publisher;
        *pub_ = n_.advertise< std_msgs::String>(topic.c_str(), queue_size);
        flag_activation = true;
	  }
	  msg_string = msg;
      pub_ -> publish(*msg_string);
	  flag_update = true;
    }
    bool sub(string topic, std_msgs::String *&msg)
	{
      if(!flag_activation)
      {
		sub_ = new ros::Subscriber;
        *sub_ = n_.subscribe(topic.c_str(), queue_size, &ROS_Comm_Cell::callBack_string, this);
        flag_activation = true;
      }
	  msg_string = msg;
    }
    void shutdown()
    {
	  if(pub_ != NULL)
	  {
	    pub_ -> shutdown();
		delete pub_;
	  }
	  if(sub_ != NULL)
	  {
	    sub_ -> shutdown();
		delete sub_;
	  }
      flag_activation = false;
    }
    void refresh()
    {
      if(pub_ != NULL)
	  {
        topic = pub_ -> getTopic();
        num_link = pub_ -> getNumSubscribers();
	  }
	  if(sub_ != NULL)
	  {
        topic = sub_ -> getTopic();
        num_link = sub_ -> getNumPublishers();
	  }
    }
	
  public:
    ROS_Comm_Cell *linkCell;
};


ROS_Comm_Cell::~ROS_Comm_Cell()
{
	shutdown();
	if(linkCell != NULL && linkCell != this)
	  delete linkCell;
}

#endif