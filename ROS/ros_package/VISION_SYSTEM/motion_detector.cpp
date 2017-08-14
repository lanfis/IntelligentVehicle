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

#include "MOTION_DETECTOR/Motion_Detector.h"
#include "MOTION_DETECTOR/Motion_Detector_KNN.h"

using namespace std;
using namespace ros;
using namespace cv;

namespace motion_detector_nodelet {
    
class motion_detector : public nodelet::Nodelet
{    
  private:
    string ver_ = "1.1";
    ros::NodeHandle n_;
    Motion_Detector_KNN *md;

  public:
    motion_detector();//ros::NodeHandle& nh);
    ~motion_detector();
    //void run();
    
  public:
    virtual void onInit()
    {
      n_ = getNodeHandle();
      md = new Motion_Detector_KNN(n_);
      md -> init();
    }
    
};

motion_detector::motion_detector()// : n_(nh)
{    
}

motion_detector::~motion_detector()
{
    delete md;
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(motion_detector_nodelet::motion_detector, nodelet::Nodelet);
