#include <cstdlib>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ROS_INFO("Initializing pic_sim_video ...");
  ros::init(argc, argv, "pic_sim_video");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("pic_sim/image_raw", 1);
  ROS_INFO("Reading %s ...", argv[1]);

  ros::Rate loop_rate(30);
  cv::VideoCapture video(argv[1]);
  cv::Mat image;// = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

  while (nh.ok()) {
  video >> image;
//  cv::waitKey(10);
  cv_bridge::CvImage cv_img(std_msgs::Header(), "bgr8", image);
  sensor_msgs::ImagePtr msg = cv_img.toImageMsg();
//  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

