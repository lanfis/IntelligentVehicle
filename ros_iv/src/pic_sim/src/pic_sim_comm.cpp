#include <cstdlib>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "../../../../ROS/ros_comm/ros_link.h"

int main(int argc, char** argv)
{
  string nodeName = "pic_sim_comm";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());
  ros::NodeHandle nh;
  
  ROS_Link ros_link(nh, nodeName);
  ros_link.pub_init();
  ros_link.sub_init();
  
  /*image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("pic_sim/image_raw", 1);
  ROS_INFO("Reading %s ...", argv[1]);
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::waitKey(10);
  cv_bridge::CvImage cv_img(std_msgs::Header(), "bgr8", image);
  sensor_msgs::ImagePtr msg = cv_img.toImageMsg();*/
//  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) 
  {
//	ros_link.status();
    //pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

