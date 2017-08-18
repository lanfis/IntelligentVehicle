#include <cstdlib>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ROS_INFO("Initializing tnn_train_test ...");
  ros::init(argc, argv, "tnn_learn_test");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("pic_sim/image_raw", 1);
  ROS_INFO("Reading %s ...", argv[1]);
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::waitKey(10);
  cv_bridge::CvImage cv_img(std_msgs::Header(), "bgr8", image);
  sensor_msgs::ImagePtr msg = cv_img.toImageMsg();
//  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

