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

#include "CAMERA_FOCUS/Camera_Focus.h"

using namespace std;
using namespace ros;
using namespace cv;

namespace camera_focus_nodelet {
	
class camera_focus : public nodelet::Nodelet
{    
  private:
    string ver_ = "1.1";
    ros::NodeHandle n_;
	Camera_Focus *cf;

  public:
    camera_focus();//ros::NodeHandle& nh);
    ~camera_focus();
    //void run();
	
  public:
    virtual void onInit()
    {
      n_ = getNodeHandle();
	  cf = new Camera_Focus(n_);
	  cf -> init();
    }
    
};

camera_focus::camera_focus()// : n_(nh)
{    
}

camera_focus::~camera_focus()
{
    delete cf;
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(camera_focus_nodelet::camera_focus, nodelet::Nodelet);
