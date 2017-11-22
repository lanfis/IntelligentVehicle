#pragma once
#ifndef _PLATFORM_CONTROL_H_
#define _PLATFORM_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <string>
#include <cstring>
#include <sstream>

#ifdef _ROS_LINK_H_
#include "../../matrix/ros_link.h"
#endif


using namespace ros;
using namespace std;



class Platform_Control
{
  private:
	#define ros_rate 100//Hz

	#define HEADER		"PC"
	#define YAW 		"Y"
	#define PITCH 		"P"
	#define ANG_MS_MIN	700
	#define ANG_MS_MAX	2300
//	#define SHUTDOWN 	"SHUTDOWN"
//	#define STATUS 		"STATUS"
	

  public:
    string nodeName = "platform_control";
    //motor
    string topic_platform_control_platform_cmd_pub = "platform_control/platform_cmd";
    string topic_platform_control_platform_status_sub = "platform_control/platform_status";
    //computer
    string topic_platform_control_cmd_yaw_sub = "platform_control/cmd_yaw";
    string topic_platform_control_cmd_pitch_sub = "platform_control/cmd_pitch";
//    string topic_platform_control_cmd_status_pub = "platform_control/cmd_status";

  private:
    float ver_ = 1.0;
	int queue_size = 4;
	#ifdef _ROS_LINK_H_
	ROS_Link *ros_link = NULL;
	#endif

  private:
    ros::NodeHandle n_;
    ros::Publisher platform_control_platform_cmd_pub_;
    ros::Subscriber platform_control_platform_status_sub_;
    ros::Subscriber platform_control_cmd_yaw_sub_;
    ros::Subscriber platform_control_cmd_pitch_sub_;

    ros::SubscriberStatusCallback connect_cb_platform_cmd;
    ros::SubscriberStatusCallback disconnect_cb_platform_cmd;
    
	void platform_cmd_publish();
    void platform_status_callBack(const std_msgs::String::ConstPtr& msg);
    void command_yaw_callBack(const std_msgs::Float32::ConstPtr& msg);
    void command_pitch_callBack(const std_msgs::Float32::ConstPtr& msg);

  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void connectCb_platform_cmd(const ros::SingleSubscriberPublisher& ssp)
    {
        if(platform_control_platform_cmd_pub_.getNumSubscribers() > 1) return;
        ROS_INFO("%s connected !", topic_platform_control_platform_cmd_pub.c_str());
  	    sub_init();
		sub_topic_get();
    }
    void disconnectCb_platform_cmd(const ros::SingleSubscriberPublisher&)
    {
        if(platform_control_platform_cmd_pub_.getNumSubscribers() > 0) return;
        ROS_WARN("%s disconnected !", topic_platform_control_platform_cmd_pub.c_str());
        sub_shutdown();
    }

  private:
	float cmd_yaw_ = 0;
	float cmd_pitch_ = 0;
	string platform_status;

  public:
    Platform_Control(ros::NodeHandle& nh);
    ~Platform_Control();
    void run();

  public:
    virtual void init()
    {
      connect_cb_platform_cmd    = boost::bind(&Platform_Control::connectCb_platform_cmd, this, _1);
      disconnect_cb_platform_cmd = boost::bind(&Platform_Control::disconnectCb_platform_cmd, this, _1);
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
	  #ifdef _ROS_LINK_H_
	  ros_link = new ROS_Link(n_, nodeName);
	  ros_link -> pub_init();
	  ros_link -> sub_init();
	  ros_link -> add_cell(platform_control_cmd_yaw_sub_, topic_platform_control_cmd_yaw_sub);
	  ros_link -> add_cell(platform_control_cmd_pitch_sub_, topic_platform_control_cmd_pitch_sub);
	  #endif
    }
};

Platform_Control::Platform_Control(ros::NodeHandle& nh) : n_(nh)
{ 
}

Platform_Control::~Platform_Control()
{
	#ifdef _ROS_LINK_H_
	if(ros_link != NULL || ros_link != this) delete ros_link;
	#endif
}

void Platform_Control::run()
{
}

void Platform_Control::platform_cmd_publish()
{
	std_msgs::String msg;
	string platform_cmd_yaw = HEADER  YAW;
	string platform_cmd_pitch = HEADER  PITCH;
	int value_yaw = cmd_yaw_ * float(ANG_MS_MAX - ANG_MS_MIN) + float(ANG_MS_MIN);
	int value_pitch = cmd_pitch_ * float(ANG_MS_MAX - ANG_MS_MIN) + float(ANG_MS_MIN);
	stringstream token;
	string str;
	token << value_yaw;
	token >> str;
	platform_cmd_yaw = platform_cmd_yaw + str;
	token.clear();
	token << value_pitch;
	token >> str;
	platform_cmd_pitch = platform_cmd_pitch + str;
	msg.data = platform_cmd_yaw + " " + platform_cmd_pitch;
	platform_control_platform_cmd_pub_.publish(msg);
}

void Platform_Control::platform_status_callBack(const std_msgs::String::ConstPtr& msg)
{ 
}


void Platform_Control::command_yaw_callBack(const std_msgs::Float32::ConstPtr& msg)
{ 
    cmd_yaw_ = (msg -> data > 1)? 1.0 : msg -> data;
    cmd_yaw_ = (cmd_yaw_ < 0)? 0 : cmd_yaw_;
}

void Platform_Control::command_pitch_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    cmd_pitch_ = (msg -> data > 1)? 1.0 : msg -> data;
    cmd_pitch_ = (cmd_pitch_ < 0)? 0 : cmd_pitch_;
}

void Platform_Control::pub_topic_get()
{
    topic_platform_control_platform_cmd_pub = platform_control_platform_cmd_pub_.getTopic();
}

void Platform_Control::pub_init()
{
    ROS_INFO("Publisher %s initiating !", topic_platform_control_platform_cmd_pub.c_str());
	platform_control_platform_cmd_pub_ = n_.advertise< std_msgs::String>(topic_platform_control_platform_cmd_pub.c_str(), queue_size);
}

void Platform_Control::pub_shutdown()
{
    ROS_WARN("Publisher %s shuting down !", topic_platform_control_platform_cmd_pub.c_str());
    platform_control_platform_cmd_pub_.shutdown();
}

void Platform_Control::sub_topic_get()
{   
    topic_platform_control_platform_status_sub = platform_control_platform_status_sub_.getTopic();
    topic_platform_control_cmd_yaw_sub = platform_control_cmd_yaw_sub_.getTopic();
    topic_platform_control_cmd_pitch_sub = platform_control_cmd_pitch_sub_.getTopic();
}

void Platform_Control::sub_init()
{
    ROS_INFO("Subscriber %s initiating !", topic_platform_control_platform_status_sub.c_str());
	platform_control_platform_status_sub_ = n_.subscribe(topic_platform_control_platform_status_sub.c_str(), queue_size, &Platform_Control::platform_status_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_platform_control_cmd_yaw_sub.c_str());
	platform_control_cmd_yaw_sub_ = n_.subscribe(topic_platform_control_cmd_yaw_sub.c_str(), queue_size, &Platform_Control::command_yaw_callBack, this);
    ROS_INFO("Subscriber %s initiating !", topic_platform_control_cmd_pitch_sub.c_str());
	platform_control_cmd_pitch_sub_ = n_.subscribe(topic_platform_control_cmd_pitch_sub.c_str(), queue_size, &Platform_Control::command_pitch_callBack, this);
}

void Platform_Control::sub_shutdown()
{
    ROS_WARN("Subscriber %s shuting down !", topic_platform_control_platform_status_sub.c_str());
    platform_control_platform_status_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_platform_control_cmd_yaw_sub.c_str());
    platform_control_cmd_yaw_sub_.shutdown();
    ROS_WARN("Subscriber %s shuting down !", topic_platform_control_cmd_pitch_sub.c_str());
    platform_control_cmd_pitch_sub_.shutdown();
}


#endif
