#pragma once
#ifndef _ROS_COMM_SYS_H_
#define _ROS_COMM_SYS_H_

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

#include "ros_comm_cell.h"

using namespace std;

class ROS_Comm_Cell;

class ROS_Comm_Init
{
  public:
    ROS_Comm_Init(int argc, char **argv, string& nodeName)
    {
      ROS_INFO("Initializing %s ...", nodeName.c_str());
      ros::init(argc, argv, nodeName.c_str(), ros::init_options::AnonymousName);
    };
};


class ROS_Comm_System
{    
  private:
    float ver_ = 1.0;
	string nodeName;
	ROS_Comm_Init ros_init;
    ros::NodeHandle n_;
    ros::AsyncSpinner spinner;
    bool flag_activation;
	int thread;

    ROS_Comm_Cell *cc;
    ROS_Comm_Cell *cc_ptr;
	
  public:
    int topic_size;
	
    ROS_Comm_Cell* searchTopic(string topic);
	void newTopic(string topic);
	void topic_pub(string topic, std_msgs::String *&msg) { cc_ptr = searchTopic(topic); if(cc_ptr != NULL) cc_ptr -> pub(topic, msg);}
	void topic_sub(string topic, std_msgs::String *&msg) { cc_ptr = searchTopic(topic); if(cc_ptr != NULL) cc_ptr -> sub(topic, msg);}
    
  public:
    ROS_Comm_System(int argc, char **argv, string nodeName, int thread);// : nodeName(nodeName), queue_size(4), flag_activation(false), ros_init(argc, argv, nodeName), spinner(thread);
    ~ROS_Comm_System();
};

ROS_Comm_System::ROS_Comm_System(int argc, char **argv, string nodeName, int thread) : nodeName(nodeName), flag_activation(false), ros_init(argc, argv, nodeName), spinner(thread)
{
    ROS_INFO("Generating %s ...", nodeName.c_str());
	topic_size = 0;
	cc = new ROS_Comm_Cell(n_);
	spinner.start();
}

ROS_Comm_System::~ROS_Comm_System()
{
	if(cc != NULL)
	{
/*
	  cc_ptr = cc -> linkCell;
	  cc -> linkCell = NULL;
	  delete cc_ptr;
*/
	  delete cc;
	}
}

void ROS_Comm_System::newTopic(string topic)
{
	ROS_Comm_Cell *cc_new = new ROS_Comm_Cell(n_);
	cc_new -> topic = topic;
	cc_ptr = cc -> linkCell;
	cc -> linkCell = cc_new;
	cc_new -> linkCell = cc_ptr;
	topic_size += 1;
}

ROS_Comm_Cell* ROS_Comm_System::searchTopic(string topic)
{
	cc_ptr = cc -> linkCell;
	for(int i = 0; i < topic_size; i++)
	{
	//	cout << cc_ptr -> topic << endl;
		if(cc_ptr -> topic != topic)
		  cc_ptr = cc_ptr -> linkCell;
	    else
		  return cc_ptr;
	}
	return NULL;
}
#endif