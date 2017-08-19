#pragma once
#ifndef _ROS_LINK_H_
#define _ROS_LINK_H_

#include <string>
#include <cstring>
#include <ros/ros.h>
#include <vector>
#include <chrono>
#include <std_msgs/String.h>

#include "config/console_format.h"

using namespace std;

#define delayTime 2
#define PING "PING"
#define ACK "ACK"
#define STATUS "STATUS"
#define STATUS_BEGIN "Begin"
#define STATUS_NODENAME "NodeName"
#define STATUS_TOPIC "Topic"
#define STATUS_END "End"

struct control_cell_publisher
{
    string topic;
    ros::Publisher* pub;
};

struct control_cell_subscriber
{
    string topic;
    ros::Subscriber* sub;
};


class ROS_Link
{    
  private:
    float ver_ = 1.1;
    ros::NodeHandle n_;
    string nodeName_;
	int queue_size_pub = 4;
	int queue_size_sub = 4;
	string topic_link_pub_ = "/link";
	string topic_link_sub_ = "/status";
	vector< control_cell_publisher > control_pub_;
	vector< control_cell_subscriber > control_sub_;
	
  private:
    boost::shared_ptr< std_msgs::String > msg_pub;
    boost::shared_ptr< std_msgs::String > msg_sub;
    ros::Publisher *pub_ = NULL;
    ros::Subscriber *sub_ = NULL;
    ros::SubscriberStatusCallback connect_cb;
    ros::SubscriberStatusCallback disconnect_cb;
	void publish(string data);
	void callBack(const std_msgs::String::ConstPtr& msg);
	void connectCb(const ros::SingleSubscriberPublisher& ssp);
	void disconnectCb(const ros::SingleSubscriberPublisher&);
	
	void callBack_search_for_publisher(const std_msgs::String::ConstPtr& msg);
	
  private: 
	bool flag_ping = false;
	bool ping_get();
	bool ping_ack();
	bool flag_status = false;
	bool status_get();
	bool status_ack();
	
  public:
    bool search_for_publisher(string& topic);
	bool ping();
	bool status();
	bool add_control_pub(string& topic, ros::Publisher& pub);
	bool add_control_sub(string& topic, ros::Subscriber& sub);
	bool erase_control_pub(string& topic);
	bool erase_control_sub(string& topic);

  private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double elapsed;
    bool flag_pub = false;
    bool flag_sub = false;
	bool flag_link = false;
    
  public:
    ROS_Link(ros::NodeHandle& n, string nodeName);
    ~ROS_Link();
	void run();
	void pub_init();
	void pub_shutdown();
	void sub_init();
	void sub_shutdown();
};

ROS_Link::ROS_Link(ros::NodeHandle& n, string nodeName) : n_(n), nodeName_(nodeName), msg_pub(new std_msgs::String), msg_sub(new std_msgs::String)
{
	topic_link_pub_ = nodeName_ + topic_link_pub_;
	topic_link_sub_ = nodeName_ + topic_link_sub_;
    
	connect_cb    = boost::bind(&ROS_Link::connectCb, this, _1);
    disconnect_cb = boost::bind(&ROS_Link::disconnectCb, this, _1);
	OUT_INFO(nodeName_.c_str(), "ROS_Link is used for fast and automatically message exchanging !");
}

ROS_Link::~ROS_Link()
{
	pub_shutdown();
	sub_shutdown();
}

void ROS_Link::run()
{
    now = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
	if(ping_get()) return;
	if(ping_ack()) return;
	if(status_get()) return;
	if(status_ack()) return;
}

bool ROS_Link::status()
{
	OUT_INFO(nodeName_.c_str(), "requesting for status ...");
	flag_status = true;
	publish(STATUS);
    start = std::chrono::high_resolution_clock::now();
	return true;
}

bool ROS_Link::status_get()
{
	if(msg_sub -> data == STATUS)
	{
	    publish(STATUS_BEGIN);
		string str = STATUS_NODENAME;
		publish(str + " : " + nodeName_);
		for(int i = 0; i < control_pub_.size(); i++)
		{
			str = STATUS_TOPIC;
		    publish(str + " : " + control_pub_[i].topic);
		}
		for(int i = 0; i < control_sub_.size(); i++)
		{
			str = STATUS_TOPIC;
		    publish(str + " : " + control_sub_[i].topic);
		}
		publish(STATUS_END);
		return true;
	}
	return false;
}

bool ROS_Link::status_ack()
{
	if(flag_status)
	{
		OUT_INFO(nodeName_.c_str(), "status received !");
		flag_status = false;
		return true;
	}
	return false;
}

bool ROS_Link::ping()
{
	OUT_INFO(nodeName_.c_str(), "pinging ...");
	flag_ping = true;
	publish(PING);
    start = std::chrono::high_resolution_clock::now();
	return true;
}

bool ROS_Link::ping_get()
{
	if(msg_sub -> data == PING)
	{
	    publish(ACK);
		return true;
	}
	return false;
}

bool ROS_Link::ping_ack()
{
	if(flag_ping)
	{
		OUT_INFO(nodeName_.c_str(), "ack received !");
	    flag_link = true;
		flag_ping = false;
		return true;
	}
	return false;
}

void ROS_Link::callBack_search_for_publisher(const std_msgs::String::ConstPtr& msg){};
bool ROS_Link::search_for_publisher(string& topic)
{
    ros::Subscriber sub_temp;
	sub_temp = n_.subscribe(topic.c_str(), 1, &ROS_Link::callBack_search_for_publisher, this);
	bool is_publisher = (sub_temp.getNumPublishers() > 0)? true : false;
	sub_temp.shutdown();
	return is_publisher;
}

bool ROS_Link::add_control_pub(string& topic, ros::Publisher& pub)
{
	for(int i = 0; i < control_pub_.size(); i++)
	{
		if(topic == control_pub_[i].topic)
			return false;
	}
	control_cell_publisher p;
	p.topic = topic;
	p.pub = &pub;
	control_pub_.emplace_back(p);
	return true;
}

bool ROS_Link::add_control_sub(string& topic, ros::Subscriber& sub)
{
	for(int i = 0; i < control_sub_.size(); i++)
	{
		if(topic == control_sub_[i].topic)
			return false;
	}
	control_cell_subscriber s;
	s.topic = topic;
	s.sub = &sub;
	control_sub_.emplace_back(s);
	return true;
}

bool ROS_Link::erase_control_pub(string& topic)
{
    int idx = 0;
	for(vector<control_cell_publisher>::iterator i = control_pub_.begin(); i != control_pub_.end(); i++)
	{
		if(topic == control_pub_[idx].topic)
		{
			control_pub_.erase(i);
			return true;
		}
		idx += 1;
	}
	return false;
}

bool ROS_Link::erase_control_sub(string& topic)
{
    int idx = 0;
	for(vector<control_cell_subscriber>::iterator i = control_sub_.begin(); i != control_sub_.end(); i++)
	{
		if(topic == control_sub_[idx].topic)
		{
			control_sub_.erase(i);
			return true;
		}
		idx += 1;
	}
	return false;
}

void ROS_Link::publish(string data)
{
	if(!flag_pub) return;
	msg_pub -> data = data;
	pub_ -> publish(msg_pub);
}

void ROS_Link::callBack(const std_msgs::String::ConstPtr& msg)
{
    this -> msg_sub -> data = msg -> data;
	run();
}

void ROS_Link::connectCb(const ros::SingleSubscriberPublisher& ssp)
{
    if(pub_ -> getNumSubscribers() > 1)
	{
		string str = topic_link_pub_ + " : "+ "links establishing !";
		OUT_WARN(nodeName_.c_str(), str);
		return;
	}
	if(!flag_sub)
	{
	  string str = topic_link_pub_ + " : "+ "establishing !";
      OUT_INFO(nodeName_.c_str(), str);
      sub_init();
	  ping();
	}
}

void ROS_Link::disconnectCb(const ros::SingleSubscriberPublisher&)
{
    if(pub_ -> getNumSubscribers() > 0) return;
	if(flag_sub)
	{
	  string str = topic_link_sub_ + " : " + "shutting down !";
      OUT_WARN(nodeName_.c_str(), str);
	  sub_shutdown();
	}
}

void ROS_Link::pub_init()
{
    pub_shutdown();
    pub_ = new ros::Publisher;
    *pub_ = n_.advertise< std_msgs::String >(topic_link_pub_.c_str(), queue_size_pub, connect_cb, disconnect_cb);
    this -> topic_link_pub_ = pub_ -> getTopic();
	flag_pub = true;
}

void ROS_Link::pub_shutdown()
{
    if(pub_ != NULL)
    {
      pub_ -> shutdown();
      delete pub_;
	  pub_ = NULL;
    }
	flag_pub = false;
}

void ROS_Link::sub_init()
{
    sub_shutdown();
    sub_ = new ros::Subscriber;
    *sub_ = n_.subscribe(topic_link_sub_.c_str(), queue_size_sub, &ROS_Link::callBack, this);
    this -> topic_link_sub_ = sub_ -> getTopic();
	flag_sub = true;
}

void ROS_Link::sub_shutdown()
{
    if(sub_ != NULL)
    {
      sub_ -> shutdown();
      delete sub_;
	  sub_ = NULL;
    }
	flag_sub = false;
}

#endif
