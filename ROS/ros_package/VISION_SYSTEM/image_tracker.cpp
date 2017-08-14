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

#include "IMAGE_TRACKER/Image_Tracker.h"

using namespace std;
using namespace ros;
using namespace cv;

namespace image_tracker_nodelet {
    
class image_tracker : public nodelet::Nodelet
{    
  private:
    string ver_ = "1.1";
    ros::NodeHandle n_;
    Image_Tracker *it;

  public:
    image_tracker();//ros::NodeHandle& nh);
    ~image_tracker();
    //void run();
    
  public:
    virtual void onInit()
    {
      n_ = getNodeHandle();
      it = new Image_Tracker(n_);
      it -> init();
    }
    
};

image_tracker::image_tracker()// : n_(nh)
{    
}

image_tracker::~image_tracker()
{
    delete it;
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_tracker_nodelet::image_tracker, nodelet::Nodelet);
