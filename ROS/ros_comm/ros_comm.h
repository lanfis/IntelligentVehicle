#pragma once
#ifndef _ROS_COMM_FRAME_H_
#define _ROS_COMM_FRAME_H_

#include <string>
#include <cstring>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "Publisher_Cell_Float32.h"
#include "Subscriber_Cell_Float32.h"


using namespace std;



class ROS_Comm_Frame
{    
  private:
    float ver_ = 1.1;
    ros::NodeHandle n_;
    string nodeName;
    
    
  public:
    int topic_size;
    
    ROS_Comm_Cell* searchTopic(string topic);
    void newTopic(string topic);
    void topic_pub(string topic, std_msgs::String *&msg) { cc_ptr = searchTopic(topic); if(cc_ptr != NULL) cc_ptr -> pub(topic, msg);}
    void topic_sub(string topic, std_msgs::String *&msg) { cc_ptr = searchTopic(topic); if(cc_ptr != NULL) cc_ptr -> sub(topic, msg);}
    
  public:
    ROS_Comm_Frame(ros::NodeHandle& n) : n_(n), nodeName(nodeName)
    ~ROS_Comm_Frame();
};

ROS_Comm_Frame::ROS_Comm_Frame(ros::NodeHandle& n) : n_(n), nodeName(nodeName)
{
    ROS_INFO("Generating %s ...", nodeName.c_str());
}

ROS_Comm_Frame::~ROS_Comm_Frame()
{
}

void ROS_Comm_Frame::newTopic(string topic)
{
    ROS_Comm_Cell *cc_new = new ROS_Comm_Cell(n_);
    cc_new -> topic = topic;
    cc_ptr = cc -> linkCell;
    cc -> linkCell = cc_new;
    cc_new -> linkCell = cc_ptr;
    topic_size += 1;
}

ROS_Comm_Cell* ROS_Comm_Frame::searchTopic(string topic)
{
    cc_ptr = cc -> linkCell;
    for(int i = 0; i < topic_size; i++)
    {
    //    cout << cc_ptr -> topic << endl;
        if(cc_ptr -> topic != topic)
          cc_ptr = cc_ptr -> linkCell;
        else
          return cc_ptr;
    }
    return NULL;
}
#endif
