#pragma once
#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
/*
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
*/
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
/*
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
*/
#include "vision_system_bridge.h"
#include "tracking_system_bridge.h"
#include "motor_system_bridge.h"
#include "reconstruction_system_bridge.h"

#include "bridge/viewer_bridge.h"

using namespace ros;
using namespace std;
using namespace cv;
//using namespace pcl;



const string nodeName = "Interface";



const string topic_adel_interface_image_pub = nodeName+"/interface_image";

class Interface
{
  private:
    float ver_;
    std::mutex lock;

    ros::NodeHandle n_;
    image_transport::ImageTransport it;
    ros::AsyncSpinner spinner;

    std::thread AdelThread;

  private:
    string system_status;
    Mat interface_image;
    string keyboard;
    int view_switch;
    
  private:
    void Interface_Interface();
    void Interface_Interface_Status();
    
  private:
    bool flag_vision_system_work;
    Vision_System_Bridge *vsb;
    bool Vision_System_Bridge_Init();
    bool Vision_System_Bridge_Check(Vision_System_Bridge* vsb);
    
    bool flag_tracking_system_work;
    Tracking_System_Bridge *tsb;
    bool Tracking_System_Bridge_Init();
    bool Tracking_System_Bridge_Check(Tracking_System_Bridge* tsb);
    void Tracking_System_Bridge_Update(Tracking_System_Bridge* tsb);

    bool flag_motor_system_work;
    Motor_System_Bridge *msb;
    bool Motor_System_Bridge_Init();
    bool Motor_System_Bridge_Check(Motor_System_Bridge* msb);
    void Motor_System_Bridge_Update(Motor_System_Bridge* msb);
    
    bool flag_reconstruction_system_work;
    Reconstruction_System_Bridge *rsb;
    bool Reconstruction_System_Bridge_Init();
    bool Reconstruction_System_Bridge_Check(Reconstruction_System_Bridge* rsb);
    void Reconstruction_System_Bridge_Update(Reconstruction_System_Bridge* rsb);

  private:
    bool flag_viewer_bridge_work;
    Viewer_Bridge *vb;
    bool Interface_Interface_Image_Init();
    bool Interface_Interface_Image_Check(Viewer_Bridge* vb);
    void Interface_Interface_Image_Update(Viewer_Bridge* vb);
    void Interface_Interface_Keyboard_Event();
    void Interface_Interface_View();

  public:
    Interface();
    ~Interface();
    void run();

  private:
    void System_Init();
    void System_Check();
    void System_Update();
/*
    void start();
    void stop();
    void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                  const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);
*/
/*                
    void cloudAdel();
    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *);
    void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const;  
    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const;
    void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue);  
    void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out);
    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const;
    void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored);
    void createLookup(size_t width, size_t height);
    
    void ros_param();
    void ros_param_get();
*/
};

Interface::Interface() : n_("~"), spinner(0), it(n_)//, vsb(n_)//, topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
        //updateCloud(false), save(false), running(false), frame(0), queueSize(5),
{
    ver_ = 1.0;
    view_switch = 0;
//    topic_interface_image = topic_adel_interface_image_pub;
    System_Init();
    spinner.start();
    run();
}

Interface::~Interface()
{
    if(vsb != NULL)        delete vsb;
    if(tsb != NULL)        delete tsb;
    if(msb != NULL)        delete msb;
    if(vb  != NULL)        delete vb;
}

void Interface::run()
{
    Interface_Interface();
    
}
//
void Interface::System_Init()
{
    system_status = "System Initializing ...";
    ROS_INFO("-- %s", system_status.c_str());
    //System Initializing
    flag_viewer_bridge_work = Interface_Interface_Image_Init();
    
    flag_vision_system_work = Vision_System_Bridge_Init();
    flag_tracking_system_work = Tracking_System_Bridge_Init();
    flag_motor_system_work = Motor_System_Bridge_Init();
    flag_reconstruction_system_work = Reconstruction_System_Bridge_Init();

    //System Checking
    System_Check();
}

void Interface::System_Check()
{
    system_status = "System Checking ...";
    ROS_INFO("-- %s", system_status.c_str());
    //System Checking    
    Vision_System_Bridge_Check(this -> vsb);
    Tracking_System_Bridge_Check(this -> tsb);
    Motor_System_Bridge_Check(this -> msb);
    Reconstruction_System_Bridge_Check(this -> rsb);

    Interface_Interface_Image_Check(this -> vb);
}

void Interface::System_Update()
{
  Motor_System_Bridge_Update(this -> msb);
  Tracking_System_Bridge_Update(this -> tsb);
  Interface_Interface_Image_Update(this -> vb);
}
//
bool Interface::Interface_Interface_Image_Init()
{
    system_status = "Initializing Interface Image ...";
    ROS_INFO("-- %s", system_status.c_str());
    this -> vb = new Viewer_Bridge(n_);
    return true;
}

bool Interface::Interface_Interface_Image_Check(Viewer_Bridge* vb)
{
    //ROS_INFO("-- Checking Interface Image ...");
    if(vb == NULL)
    {
      system_status = "Interface Image unpredictable failure !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    if(!vb -> flag_update_image)
    {
      system_status = "Interface Image updating fail !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    return true;
}

void Interface::Interface_Interface_Image_Update(Viewer_Bridge* vb)
{
  vb -> update_image = true;
  vb -> run();
}

bool Interface::Reconstruction_System_Bridge_Init()
{
    system_status = "Initializing Reconstruction System ...";
    ROS_INFO("-- %s", system_status.c_str());
    this -> rsb = new Reconstruction_System_Bridge(n_);
    return true;
}

bool Interface::Reconstruction_System_Bridge_Check(Reconstruction_System_Bridge* rsb)
{
    //ROS_INFO("-- Checking Reconstruction System ...");
    if(rsb == NULL)
    {
      system_status = "Reconstruction System unpredictable failure !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    if(!rsb -> flag_update_disparity_mono_image)
    {
      system_status = "Reconstruction System disparity mono image updating fail !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    return true;
}

bool Interface::Motor_System_Bridge_Init()
{
    system_status = "Initializing Motor System ...";
    ROS_INFO("-- %s", system_status.c_str());
    this -> msb = new Motor_System_Bridge(n_);
    Motor_System_Bridge_Update(this -> msb);
    return true;
}

bool Interface::Motor_System_Bridge_Check(Motor_System_Bridge* msb)
{
    //ROS_INFO("-- Checking Motor System ...");
    if(msb == NULL)
    {
      system_status = "Motor System unpredictable failure !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    if(!msb -> flag_update_velocity)
    {
      system_status = "Motor System velocity updating fail !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    if(!msb -> flag_update_rotate)
    {
      system_status = "Motor System rotate updating fail !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    return true;
}

void Interface::Motor_System_Bridge_Update(Motor_System_Bridge* msb)
{
  #define MOTOR_STATUS "STATUS"
  msb -> motor_command = MOTOR_STATUS;
  msb -> update_command = true;
  msb -> run();
}
   
bool Interface::Tracking_System_Bridge_Init()
{
    system_status = "Initializing Tracking System ...";
    ROS_INFO("-- %s", system_status.c_str());
    this -> tsb = new Tracking_System_Bridge(n_);
    tsb -> activation_face = true;
    tsb -> activation_disparity = false;
    return true;
}

bool Interface::Tracking_System_Bridge_Check(Tracking_System_Bridge* tsb)
{
    //ROS_INFO("-- Checking Tracking System ...");
    if(tsb == NULL)
    {
      system_status = "Tracking System unpredictable failure !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    if(!tsb -> flag_update_image)
    {
      system_status = "Tracking System image tracked updating fail !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    if(!tsb -> flag_update_status)
    {
      system_status = "Tracking System tracking status updating fail !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    if(!tsb -> flag_update_region)
    {
      system_status = "Tracking System tracking region updating fail !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    return true;
}
    
void Interface::Tracking_System_Bridge_Update(Tracking_System_Bridge* tsb)
{
  tsb -> run();
}

bool Interface::Vision_System_Bridge_Init()
{
    system_status = "Initializing Vision System ...";
    ROS_INFO("-- %s", system_status.c_str());
    this -> vsb = new Vision_System_Bridge(n_);
    return true;
}
    
bool Interface::Vision_System_Bridge_Check(Vision_System_Bridge* vsb)
{
    //ROS_INFO("-- Checking Vision System ...");
    if(vsb == NULL)
    {
      system_status = "Vision System unpredictable failure !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    if(!vsb -> flag_update_image)
    {
      system_status = "Vision System color image updating fail !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    if(!vsb -> flag_update_depth)
    {
      system_status = "Vision System image depth updating fail !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    if(!vsb -> flag_update_camera_info)
    {
      system_status = "Vision System camera info updating fail !";
      ROS_WARN("---- %s", system_status.c_str());
      return false;
    }
    return true;
}


void Interface::Interface_Interface()
{
  System_Update();
  Interface_Interface_Status();
  Interface_Interface_Keyboard_Event();
  Interface_Interface_View();
  System_Update();
}

void Interface::Interface_Interface_Status()
{
  ostringstream status;
  status.str("");

  status << "Motor Velocity / Rotate : ";
  if(msb -> flag_update_velocity)    status << msb -> velocity;
  else                               status << "ERR";
  status << " / ";
  if(msb -> flag_update_rotate  )    status << msb -> rotate;
  else                               status << "ERR";
  
  vb -> text = status.str();
  vb -> image = interface_image;
  this -> keyboard = vb -> keyboard;
  vb -> system_status = system_status;

  //Interface_Interface_Image_Update(vb);
  //System_Check();
}

void Interface::Interface_Interface_Keyboard_Event()
{
	if(this -> keyboard == "v")
        {
          view_switch = 0;
          vsb -> activation = true;
          tsb -> activation = false;
        }
	if(this -> keyboard == "t")
        {
          view_switch = 1;
          vsb -> activation = false;
          tsb -> activation = true;
		  
  if(tsb -> tracking_lock)
  {
    tsb -> activation_face = false;
  }
  else
  {
    tsb -> activation_face = true;
  }
  
  //tsb -> activation_disparity = true;
        }
}
  
void Interface::Interface_Interface_View()
{
	switch(this -> view_switch)
	{
		case 0:
                  interface_image = vsb -> image;
		  break;
		case 1:
                  interface_image = tsb -> image;
		  break;
	}
}

int main(int argc, char **argv)
{
  ROS_INFO("Initializing %s ...", nodeName.c_str());
 /*
#if EXTENDED_OUTPUT
  ROSCONSOLE_AUTOINIT;
  if(!getenv("ROSCONSOLE_FORMAT"))
  {
    ros::console::g_formatter.tokens_.clear();
    ros::console::g_formatter.init("[${severity}] ${message}");
  }
#endif
*/
  ros::init(argc, argv, nodeName.c_str(), ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }
  
  
  ROS_INFO("Generating %s ...", nodeName.c_str());
  Interface adel;
  while(ros::ok())
  {
    adel.run();
  }

  ros::shutdown();
  return 0;
}

#endif
