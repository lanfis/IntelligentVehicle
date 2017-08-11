#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/UInt16MultiArray.h>

#include "CAMERA/Camera.h"

using namespace std;
using namespace ros;
using namespace cv;

namespace vision_system {
class camera_nodelet : public nodelet::Nodelet
{    
  private:
    string ver_ = "1.1";
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::ImageTransport it_roi_;
    sensor_msgs::ImagePtr msg_image;
    sensor_msgs::ImagePtr msg_image_roi;
    sensor_msgs::CameraInfo camera_info_;
      
    image_transport::Publisher it_pub_;
    image_transport::Publisher it_roi_pub_;
    ros::Subscriber roi_ratio_sub_;
    ros::Subscriber roi_width_offset_sub_;
    ros::Subscriber roi_height_offset_sub_;
    ros::Publisher camera_info_pub_;
    ros::Subscriber device_id_sub_;
    ros::Subscriber resolution_ratio_sub_;
    ros::Subscriber activation_sub_;
    ros::Publisher status_pub_;
    
    int queue_size;
      
  public:
    string nodeName;
    string topic_image_pub;
    string topic_image_roi_pub;
    string topic_image_roi_ratio_sub;
    string topic_image_roi_width_offset_sub;
    string topic_image_roi_height_offset_sub;
    string topic_camera_info_pub;
    string topic_device_id_sub;
    string topic_resolution_ratio_sub;
    string topic_activation_sub;
    string topic_status_pub;
      
    int device_id;
    Mat *image;
    Mat image_roi;
    float image_roi_ratio;
    int image_roi_width_offset;
    int image_roi_height_offset;
     
    bool activation;
      
  private:
    Camera *camera;
    bool flag_activation;
    void image_roi_get();
    void roi_ratio_callBack(const std_msgs::Float32::ConstPtr& msg);
    void roi_width_offset_callBack(const std_msgs::Int32::ConstPtr& msg);
    void roi_height_offset_callBack(const std_msgs::Int32::ConstPtr& msg);
    void device_id_callBack(const std_msgs::Int32::ConstPtr& msg);
    void resolution_ratio_callBack(const std_msgs::Float32::ConstPtr& msg);
    void activation_callBack(const std_msgs::Bool::ConstPtr& msg);
    void create_Camera_Info();
    void create_Camera_Info(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const;

  public:
    camera_nodelet();
    ~camera_nodelet();
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void status_publish(string status);
    void image_roi_publish();
    void image_publish();
    void camera_info_publish();
    void run();
	
  public:
    virtual void onInit()
    {
      n_ = getNodeHandle();
	  //pub_init();
	  //sub_init();
    }
};

camera_nodelet::camera_nodelet() : it_(n_), it_roi_(n_), msg_image(new sensor_msgs::Image), msg_image_roi(new sensor_msgs::Image)
{
    nodeName = "camera_nodelet";
    topic_image_pub = nodeName+"/image";
    topic_image_roi_pub = nodeName+"/image_roi";
    topic_image_roi_ratio_sub = nodeName+"/image_roi_ratio";
    topic_image_roi_width_offset_sub = nodeName+"/image_roi_width_offset";
    topic_image_roi_height_offset_sub = nodeName+"/image_roi_height_offset";
    topic_camera_info_pub = nodeName+"/camera_info";
    topic_device_id_sub = nodeName+"/device_id";
    topic_resolution_ratio_sub = nodeName+"/resolution_ratio";
    topic_activation_sub = nodeName+"/activation";
    topic_status_pub = nodeName+"/status";
    
    queue_size = 4;
    
    camera = new Camera;
    this -> image = &camera -> image;
    if(camera -> is_open())
      ROS_INFO("Camera initializing ...ok !");
    else
      ROS_WARN("Camera initializing ...fail !");
    
    image_roi_ratio = 0.5;
    image_roi_width_offset = 0;
    image_roi_height_offset = 0;
      
    activation = true;
    flag_activation = true;
    pub_init();
    pub_topic_get();
    sub_init();
    sub_topic_get();
    status_publish(nodeName + " initializing ...");
    run();
}

camera_nodelet::~camera_nodelet()
{
//    delete msg_image;
//    delete msg_image_roi;
}

void camera_nodelet::run()
{
	while(flag_activation)
	{
    if(!activation)
    {
        if(flag_activation)
        {
//          pub_shutdown();
//          sub_shutdown();
          flag_activation = false;
        }
        return;
    }
    if(!flag_activation)
    {
//      pub_init();
//      sub_init();
      flag_activation = true;
    }
    
    if(!camera -> is_open())
    {
        ROS_WARN("Opening camera source %d error !", camera -> device_id);
        return;
    }
    if(camera -> run()) 
    {
        image_roi_get();
        image_publish();
        image_roi_publish();
    }
    camera_info_publish();
	}
}

void camera_nodelet::image_roi_get()
{
    int center_col = (*image).cols/2;
    int center_row = (*image).rows/2;
    int roi_width =  (*image).cols*image_roi_ratio;
    int roi_height = (*image).cols*image_roi_ratio;
    int col_s = ((center_col - roi_width/2  + image_roi_width_offset) < 0)? 0 : center_col - roi_width/2  + image_roi_width_offset;
    int col_e = ((center_col + roi_width/2  + image_roi_width_offset) > (*image).cols)? (*image).cols : center_col + roi_width/2  + image_roi_width_offset;
    int row_s = ((center_row - roi_height/2 + image_roi_height_offset)< 0)? 0 : center_row - roi_height/2 + image_roi_height_offset;
    int row_e = ((center_row + roi_height/2 + image_roi_height_offset)> (*image).rows)? (*image).rows : center_row + roi_height/2 + image_roi_height_offset;
    image_roi = (*image)(Range(row_s, row_e), Range(col_s, col_e));
}

void camera_nodelet::image_roi_publish()
{
  //sensor_msgs::ImagePtr msg_image_roi(new sensor_msgs::Image);
  msg_image_roi = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_roi).toImageMsg();
  //sensor_msgs::ImagePtr msg_image_roi;
  //msg_image_roi = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_roi).toImageMsg();
  it_roi_pub_.publish(msg_image_roi);
}

void camera_nodelet::image_publish()
{
  //sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image);
  msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
  //sensor_msgs::ImagePtr msg_image;
  //msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
  it_pub_.publish(msg_image);
}

void camera_nodelet::camera_info_publish()
{
  create_Camera_Info();
  camera_info_pub_.publish(camera_info_);
}

void camera_nodelet::create_Camera_Info()
{
  cv::Size size;
  size.width = camera -> width;
  size.height = camera -> height;

  create_Camera_Info(size, camera -> cameraMatrix, camera -> distortion, cv::Mat::eye(3, 3, CV_64F), camera -> projection, camera_info_);
}

void camera_nodelet::create_Camera_Info(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const
{
  cameraInfo.height = size.height;
  cameraInfo.width = size.width;

  const double *itC = cameraMatrix.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    cameraInfo.K[i] = *itC;
  }

  const double *itR = rotation.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itR)
  {
    cameraInfo.R[i] = *itR;
  }

  const double *itP = projection.ptr<double>(0, 0);
  for(size_t i = 0; i < 12; ++i, ++itP)
  {
    cameraInfo.P[i] = *itP;
  }

  cameraInfo.distortion_model = "plumb_bob";
  cameraInfo.D.resize(distortion.cols);
  const double *itD = distortion.ptr<double>(0, 0);
  for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
  {
    cameraInfo.D[i] = *itD;
  }
}

void camera_nodelet::roi_ratio_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("ROI ratio is changed to %f !", msg -> data);
    image_roi_ratio = (msg -> data < 1)? msg -> data : 1;
}

void camera_nodelet::roi_width_offset_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("ROI width offset is changed to %d !", msg -> data);
    image_roi_width_offset = msg -> data;
}

void camera_nodelet::roi_height_offset_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("ROI height offset is changed to %d !", msg -> data);
    image_roi_height_offset = msg -> data;
}

void camera_nodelet::device_id_callBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_WARN("Camera ID is changed to %d !", msg -> data);
    camera -> device_id = msg -> data;
}

void camera_nodelet::resolution_ratio_callBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_WARN("Resolution ratio is changed to %f !", msg -> data);
    camera -> resolution_ratio = msg -> data;
}

void camera_nodelet::activation_callBack(const std_msgs::Bool::ConstPtr& msg)
{
    activation = msg -> data;
    if(activation)
    {
      ROS_WARN("%s activated !", nodeName.c_str());
      status_publish(nodeName + " activated !");
    }
    else
    {
      ROS_WARN("%s inactivated !", nodeName.c_str());
      status_publish(nodeName + " inactivated !");
    }
    this -> run();
}

void camera_nodelet::status_publish(string status)
{
    std_msgs::String msg;
    msg.data = status.c_str();
    status_pub_.publish(msg);
}

void camera_nodelet::pub_topic_get()
{
    topic_image_pub = it_pub_.getTopic();
    topic_image_roi_pub = it_roi_pub_.getTopic();
    topic_camera_info_pub = camera_info_pub_.getTopic();
    topic_status_pub = status_pub_.getTopic();
}

void camera_nodelet::pub_init()
{
    it_pub_ = it_.advertise(topic_image_pub.c_str(), queue_size);
    it_roi_pub_ = it_roi_.advertise(topic_image_roi_pub.c_str(), queue_size);
    camera_info_pub_ = n_.advertise<sensor_msgs::CameraInfo>(topic_camera_info_pub.c_str(), queue_size);
    status_pub_ = n_.advertise<std_msgs::String>(topic_status_pub.c_str(), queue_size);
}

void camera_nodelet::pub_shutdown()
{
    it_pub_.shutdown();
    it_roi_pub_.shutdown();
    camera_info_pub_.shutdown();
}

void camera_nodelet::sub_topic_get()
{   
    topic_image_roi_ratio_sub = roi_ratio_sub_.getTopic();
    topic_image_roi_width_offset_sub = roi_width_offset_sub_.getTopic();
    topic_image_roi_height_offset_sub = roi_height_offset_sub_.getTopic();
    topic_device_id_sub = device_id_sub_.getTopic();
    topic_resolution_ratio_sub = resolution_ratio_sub_.getTopic();
    topic_activation_sub = activation_sub_.getTopic();
}

void camera_nodelet::sub_init()
{
    roi_ratio_sub_ = n_.subscribe(topic_image_roi_ratio_sub.c_str(), queue_size, &camera_nodelet::roi_ratio_callBack, this);
    roi_width_offset_sub_ = n_.subscribe(topic_image_roi_width_offset_sub.c_str(), queue_size, &camera_nodelet::roi_width_offset_callBack, this);
    roi_height_offset_sub_ = n_.subscribe(topic_image_roi_height_offset_sub.c_str(), queue_size, &camera_nodelet::roi_height_offset_callBack, this);
    device_id_sub_ = n_.subscribe(topic_device_id_sub.c_str(), queue_size, &camera_nodelet::device_id_callBack, this);
    resolution_ratio_sub_ = n_.subscribe(topic_resolution_ratio_sub.c_str(), queue_size, &camera_nodelet::resolution_ratio_callBack, this);
    activation_sub_ = n_.subscribe(topic_activation_sub.c_str(), queue_size, &camera_nodelet::activation_callBack, this);
}

void camera_nodelet::sub_shutdown()
{
    roi_width_offset_sub_.shutdown();
    roi_height_offset_sub_.shutdown();
    device_id_sub_.shutdown();
    resolution_ratio_sub_.shutdown();
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vision_system::camera_nodelet, nodelet::Nodelet);
