#include <ros/ros.h>
#include <ros/spinner.h>
#include <nodelet/loader.h>
#include <string>
#include "VISION_SYSTEM/vision_system.h"
//#include "VISION_SYSTEM/camera_focus.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "vision_system";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());
  ros::NodeHandle n;

  ROS_INFO("%s activating ok !", nodeName.c_str());
  Vision_System vs(n, 0);
  
  nodelet::Loader nodelet_loader;
  nodelet::M_string remap_camera_focus(ros::names::getRemappings());
  nodelet::V_string nargv_camera_focus;
  std::string nodelet_camera_focus_name = "camera_focus";// = ros::this_node::getName();// + "/camera_focus_nodelet/camera_focus";
  nodelet_loader.load(nodelet_camera_focus_name, "camera_focus_nodelet/camera_focus", remap_camera_focus, nargv_camera_focus);
  ROS_INFO("%s activating ok !", nodelet_camera_focus_name.c_str());
  
  nodelet::M_string remap_motion_detector(ros::names::getRemappings());
  nodelet::V_string nargv_motion_detector;
  std::string nodelet_motion_detector_name = "motion_detector";//ros::this_node::getName();// + "/motion_detector_nodelet/motion_detector";
  nodelet_loader.load(nodelet_motion_detector_name, "motion_detector_nodelet/motion_detector", remap_motion_detector, nargv_motion_detector);
  ROS_INFO("%s activating ok !", nodelet_motion_detector_name.c_str());
  
  nodelet::M_string remap_image_tracker(ros::names::getRemappings());
  nodelet::V_string nargv_image_tracker;
  std::string nodelet_image_tracker_name = "image_tracker";//ros::this_node::getName();// + "/motion_detector_nodelet/motion_detector";
  nodelet_loader.load(nodelet_image_tracker_name, "image_tracker_nodelet/image_tracker", remap_image_tracker, nargv_image_tracker);
  ROS_INFO("%s activating ok !", nodelet_image_tracker_name.c_str());
  
  nodelet::M_string remap_face_detector(ros::names::getRemappings());
  nodelet::V_string nargv_face_detector;
  std::string nodelet_face_detector_name = "face_detector";
  nodelet_loader.load(nodelet_face_detector_name, "face_detector_nodelet/face_detector", remap_face_detector, nargv_face_detector);
  ROS_INFO("%s activating ok !", nodelet_face_detector_name.c_str());
  
  while (ros::ok()) 
  {
    vs.run();
  }
  //ros::spin();
  
  ros::shutdown();
  return 0;
}

/*
//#include "ros/ros.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "camera_nodelet");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "vision_system/camera_nodelet", remap, nargv);
  //ros::spin();
  
  ros::AsyncSpinner spinner(0);
  spinner.start();
  while(ros::ok())
  {
  }
    ros::shutdown();
  return 0;
  }
  */
