#pragma once
#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <ros/ros.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <vector>
#include <chrono>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/RegionOfInterest.h>
/*
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/UInt16MultiArray.h>
*/

#include "../../matrix/ros_link.h"
#include "../../matrix/cell/Publisher_Cell_Float32.h"
#include "../../matrix/cell/Subscriber_Cell_Float32.h"

using namespace std;
using namespace ros;

#define LINK_SIZE 2
#define CELL_SIZE LINK_SIZE*2
#define UPDATE_CYCLE 128
#define UPDATE_DURATION 1000

class Matrix
{    
	public:
	    string nodeName_ = "Matrix";
    
	private:
    	string ver_ = "1.0";
	    ros::NodeHandle n_;
		int queue_size_pub_ = 4;
		int queue_size_sub_ = 4;
	    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
		double elapsed;
		int update_cycle = UPDATE_CYCLE;

    
	private:
		ROS_Link **link_;
/*		Publisher_Cell_Float32 **cell_pub_;
		int size_cell_pub__ = 0;
		Subscriber_Cell_Float32 **cell_sub_;
		int size_cell_sub__ = 0;*/

		int size_cell_pub_ = 0;
		int size_cell_sub_ = 0;
		Publisher_Cell_Float32 cell_pub_[CELL_SIZE];
		Subscriber_Cell_Float32 cell_sub_[CELL_SIZE];
		boost::shared_ptr< std_msgs::Float32 > msg_cell_pub_[CELL_SIZE];
		boost::shared_ptr< std_msgs::Float32 > msg_cell_sub_[CELL_SIZE];

	private:
		bool add_cell_pub(string topic);
		bool add_cell_sub(string topic);
//		bool erase_cell(string& topic);
		bool search_cell_pub(string& topic);
		bool search_cell_sub(string& topic);
		bool search_cell_pub(string& topic, int& idx);
		bool search_cell_sub(string& topic, int& idx);
//		float ratio = 0;
    
	private:

	public:
	    Matrix(ros::NodeHandle& nh);
	    ~Matrix();
		void construct_cell_to_link();
	    void check_link();
	    void update_link();
	    void run();
    
	public:
	    virtual void init()
	    {
			for(int i = 0; i < LINK_SIZE; i++)
			{
				link_[i] -> pub_init();
			}
	    }
    
};

Matrix::Matrix(ros::NodeHandle& nh) : n_(nh)
{    
	link_ = new ROS_Link *[LINK_SIZE];

	stringstream token;
	string idx;
	for(int i = 0; i < LINK_SIZE; i++)
	{
		token.clear();
		token << i;
		token >> idx;
		link_[i] = new ROS_Link(n_, nodeName_);
		link_[i] -> topic_link_pub_ = nodeName_ + "/link/" + idx;
		link_[i] -> topic_link_sub_ = nodeName_ + "/status/" + idx;
		link_[i] -> queue_size_pub = queue_size_pub_;
		link_[i] -> queue_size_sub = queue_size_sub_;
	}
    start = std::chrono::high_resolution_clock::now();
}

Matrix::~Matrix()
{
	for(int i = 0; i < LINK_SIZE; i++)
		delete link_[i];
	delete [] link_;
/*	for(int i = 0; i < size_cell_pub__; i++)
		delete cell_pub_[i];
	delete [] cell_pub_;
	for(int i = 0; i < size_cell_sub__; i++)
		delete cell_sub_[i];
	delete [] cell_sub_;*/
}

void Matrix::run()
{
    now = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
	if(elapsed > UPDATE_DURATION && update_cycle > 0)
	{
		update_link();
		start = std::chrono::high_resolution_clock::now();
		update_cycle -= 1;
		construct_cell_to_link();
/*	ratio = (ratio + 0.1 > 1.0)? 0.1 : ratio + 0.1;
	int idx;
	string top = "/Camera/resolution_ratio";
	if(search_cell_pub(top, idx))
		cell_pub_[idx].publish(ratio);*/
	}
}

void Matrix::construct_cell_to_link()
{
	for(int i = 0; i < LINK_SIZE; i++)
	{
//		int size_link_pub = link_[i] -> get_link_pub_size();
//		int size_link_sub = link_[i] -> get_link_sub_size();
		for(int j = 0; j < link_[i] -> get_link_pub_size(); j++)
			add_cell_pub(link_[i] -> get_link_sub_topic(j));
		for(int j = 0; j < link_[i] -> get_link_sub_size(); j++)
			add_cell_sub(link_[i] -> get_link_pub_topic(j));
	}
}

void Matrix::check_link()
{
	for(int i = 0; i < LINK_SIZE; i++)
		if(!link_[i] -> is_link())
			link_[i] -> ping();
}

void Matrix::update_link()
{
	for(int i = 0; i < LINK_SIZE; i++)
		if(link_[i] -> is_link())
			link_[i] -> status();
}

bool Matrix::add_cell_pub(string topic)
{
	if(search_cell_pub(topic)) return false;
	cell_pub_[size_cell_pub_].init(n_, topic, this -> queue_size_pub_, msg_cell_pub_[size_cell_pub_]);
	size_cell_pub_ += 1;
	for(int i = 0; i < size_cell_pub_; i++)
		cout << cell_pub_[i].getTopic() << endl;
	return true;
}

bool Matrix::add_cell_sub(string topic)
{
	if(search_cell_sub(topic)) return false;
	cell_sub_[size_cell_sub_].init(n_, topic, this -> queue_size_sub_, msg_cell_sub_[size_cell_sub_]);
	size_cell_sub_ += 1;
	for(int i = 0; i < size_cell_sub_; i++)
		cout << cell_sub_[i].getTopic() << endl;
	return true;
}
/*
bool Matrix::erase_cell(string& topic)
{
	int idx = 0;
	for(vector<Publisher_Cell_Float32>::iterator i = cell_pub_.begin(); i != cell_pub_.end(); i++, idx++)
	{
		if(topic == cell_pub_[idx].getTopic())
		{
			cell_pub_.erase(i);
			return true;
		}
	}
	idx = 0;
	for(vector<Subscriber_Cell_Float32>::iterator i = cell_sub_.begin(); i != cell_sub_.end(); i++, idx++)
	{
		if(topic == cell_sub_[idx].getTopic())
		{
			cell_sub_.erase(i);
			int jdx = 0;
			for(vector< boost::shared_ptr< std_msgs::Float32 > >::iterator j = msg_cell_sub_.begin(); j!= msg_cell_sub_.end(); j++, jdx++)
			{
				if(jdx == idx)
				{
					msg_cell_sub_.erase(j);
					return true;
				}
			}
		}
	}
	return false;
}
*/
bool Matrix::search_cell_pub(string& topic)
{
	for(int idx = 0; idx < size_cell_pub_; idx++)
		if(cell_pub_[idx].getTopic() == topic)
			return true;
	return false;
}

bool Matrix::search_cell_sub(string& topic)
{
	for(int idx = 0; idx < size_cell_sub_; idx++)
		if(cell_sub_[idx].getTopic() == topic)
			return true;
	return false;
}

bool Matrix::search_cell_pub(string& topic, int& idx)
{
	for(int idx = 0; idx < size_cell_pub_; idx++)
		if(cell_pub_[idx].getTopic() == topic)
			return true;
	return false;
}

bool Matrix::search_cell_sub(string& topic, int& idx)
{
	for(int idx = 0; idx < size_cell_sub_; idx++)
		if(cell_sub_[idx].getTopic() == topic)
			return true;
	return false;
}


#endif

