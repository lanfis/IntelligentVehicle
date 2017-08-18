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

#include "FACE_DETECTOR/Face_Detector.h"

using namespace std;
using namespace ros;
using namespace cv;

namespace face_detector_nodelet {
    
class face_detector : public nodelet::Nodelet
{    
  private:
    string ver_ = "1.1";
    ros::NodeHandle n_;
    Face_Detector *fd;

  public:
    face_detector();//ros::NodeHandle& nh);
    ~face_detector();
    //void run();
    
  public:
    virtual void onInit()
    {
      n_ = getNodeHandle();
      fd = new Face_Detector(n_);
      fd -> init();
    }
    
};

face_detector::face_detector()// : n_(nh)
{    
}

face_detector::~face_detector()
{
    delete fd;
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(face_detector_nodelet::face_detector, nodelet::Nodelet);
