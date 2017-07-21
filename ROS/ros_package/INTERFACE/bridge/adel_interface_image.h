#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

using namespace ros;
using namespace std;
using namespace cv;


class Adel_Interface_Image
{
  private:
    float ver_;
    ros::NodeHandle n_;
    image_transport::ImageTransport it;
    image_transport::Publisher it_interface_image_pub;
    
    int queue_size;
    int keyboard_sensity;
    cv_bridge::CvImagePtr cv_ptr;
    
    
  private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double elapsed;
    void pub_init();

  public:
    string topic_interface_image;
    string window_name = "Adel Viewer";

    bool activation;
    Mat image;
    Mat image_resized;
    int image_width;
    int image_height;

    string text;
    Point text_position;
    Scalar text_color;
    double text_size;
    string keyboard;
    
    bool flag_update_image;
    bool update_image;
    bool flag_window;
    bool window_show;
    
    string system_status;
    Point system_status_position;
    Scalar system_status_color;
    double system_status_size;

  private:
    void image_pub();
    void image_size_adjust();
    void text_put();
    char keyboard_read();
    void frame_rate_show();
    void system_status_put();
    
  public:
    Adel_Interface_Image(ros::NodeHandle& n);
    ~Adel_Interface_Image();
    void run();
};

Adel_Interface_Image::Adel_Interface_Image(ros::NodeHandle& n) : n_(n), it(n_)
{
  ver_ = 1.0;
  topic_interface_image = "adel_interface/image";

  image_width = 640;
  image_height = 480;

  text = "\0";
  text_position = Point(0, 20);
  text_color = Scalar(0, 255, 0);
  text_size = 0.5;
  
  system_status = "\0";
  system_status_position = Point(0, image_height-10);
  system_status_color = Scalar(0, 0, 255);
  system_status_size = 0.5;

  queue_size = 4;
  keyboard_sensity = 50;
  update_image = false;
  window_show = true;
  activation = true;
  run();
}

Adel_Interface_Image::~Adel_Interface_Image()
{
}

void Adel_Interface_Image::run()
{
  if(!activation) return;
  if(!flag_window)
  {
    if(window_show)
    {
      cv::namedWindow(window_name.c_str());
      flag_window = true;
    }
  }
  else
  {
    if(!window_show)
    {
      cv::destroyWindow(window_name.c_str());
      flag_window = false;
    }
  }
  if(!update_image) return;
//  now = std::chrono::high_resolution_clock::now();
//  elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
  
  if(flag_window) this -> keyboard = keyboard_read();
  text_put();
  system_status_put();
//  frame_rate_show();
  image_pub();
//  start = std::chrono::high_resolution_clock::now();
}

void Adel_Interface_Image::pub_init()
{  
  it_interface_image_pub = it.advertise(topic_interface_image.c_str(), queue_size);
}


void Adel_Interface_Image::image_pub()
{
  sensor_msgs::ImagePtr msg;
  if(!this -> image.empty()) 
  {
    image_size_adjust();
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this -> image_resized).toImageMsg();
    it_interface_image_pub.publish(msg);
    flag_update_image = true;
    if(flag_window)  cv::imshow(window_name.c_str(), this -> image_resized);
  }
  update_image = false;
}

void Adel_Interface_Image::image_size_adjust()
{
  if(image_width != this -> image.cols || image_height != this -> image.rows)
    resize(this -> image, this -> image_resized, Size(image_width, image_height));
  else
    this -> image.copyTo(this -> image_resized);
}

void Adel_Interface_Image::text_put()
{
  putText(this -> image
        , this -> text
        , this -> text_position
        , FONT_HERSHEY_SIMPLEX//FONT_HERSHEY_COMPLEX_SMALL
        , this -> text_size
        , this -> text_color
        , 1     //thickness – Thickness of the lines used to draw a text.
        , CV_AA //lineType – Line type. See the line for details.
        , false);//bottomLeftOrigin – When true, the image data origin is at the bottom-left corner. Otherwise, it is at the top-left corner.
}

char Adel_Interface_Image::keyboard_read()
{
  int key = cv::waitKey(keyboard_sensity);
  this -> keyboard[0] = char(key & 0xFF);
  return char(key & 0xFF);
}

void Adel_Interface_Image::frame_rate_show()
{
  ostringstream oss;
  oss.str("");
  oss << "Time Delay : " << elapsed << " ms";
  putText(this -> image
        , oss.str()
        , Point(0, image_height-10)
        , FONT_HERSHEY_SIMPLEX//FONT_HERSHEY_COMPLEX_SMALL
        , 0.5
        , Scalar(0, 0, 255)
        , 1     //thickness – Thickness of the lines used to draw a text.
        , CV_AA //lineType – Line type. See the line for details.
        , false);
}

void Adel_Interface_Image::system_status_put()
{
  if(system_status.length() == 0)
    return;
  string ss;
  ss = "[System] : " + system_status;
  putText(this -> image
        , ss
        , this -> system_status_position
        , FONT_HERSHEY_SIMPLEX//FONT_HERSHEY_COMPLEX_SMALL
        , this -> system_status_size
        , this -> system_status_color
        , 1     //thickness – Thickness of the lines used to draw a text.
        , CV_AA //lineType – Line type. See the line for details.
        , false);
}