#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <tiny_dnn/tiny_dnn.h>
#include <vector>

using namespace std;
using namespace cv;
using namespace tiny_dnn;
using namespace tiny_dnn::activation;
using namespace tiny_dnn::layers;

#include <cstdlib>
#include <string>
#include <cstring>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
using namespace boost::filesystem;
// convert image to vec_t
void convert_image(const std::string& imagefilename,
                   double scale,
                   int w,
                   int h,
                   std::vector<vec_t>& data)
{
    auto img = cv::imread(imagefilename, cv::IMREAD_GRAYSCALE);
    if (img.data == nullptr) return; // cannot open, or it's not an image

    cv::Mat_<uint8_t> resized;
    cv::resize(img, resized, cv::Size(w, h));
    vec_t d;

    std::transform(resized.begin(), resized.end(), std::back_inserter(d),
                   [=](uint8_t c) { return c * scale; });
    data.push_back(d);
}

// convert all images found in directory to vec_t
void convert_images(const std::string& directory,
                    double scale,
                    int w,
                    int h,
                    std::vector<vec_t>& data)
{
    path dpath(directory);

    BOOST_FOREACH(const path& p, 
                  std::make_pair(directory_iterator(dpath), directory_iterator())) {
        if (is_directory(p)) continue;
        convert_image(p.string(), scale, w, h, data);
    }
}


class TNN
{
  private:
    network<sequential> nn_;
    adagrad optimizer_;
    Mat img;
    vector<vec_t> train_images_;
    vector<label_t> train_labels_;
  public:
    TNN();
    ~TNN();
};

TNN::TNN()
{
// add layers
    nn_ << conv_1<tan_h>(32, 32, 5, 1, 6)  // in:32x32x1, 5x5conv, 6fmaps
        << ave_pool_1<tan_h>(28, 28, 6, 2) // in:28x28x6, 2x2pooling
        << fc<tan_h>(14 * 14 * 6, 120)   // in:14x14x6, out:120
        << fc<identity>(120, 10);        // in:120,     out:10

//    assert(net.in_data_size() == 32 * 32);
//    assert(net.out_data_size() == 10);
}

bool TNN::train_()
{
// minibatch=50, epoch=20
  return nn_.train<mse>(optimizier_, train_images_, train_labels_, 50, 20);
}

/*
class Filter2D
{
  private:
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Publisher it_pub_;
    image_transport::Subscriber it_sub_;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;
  public:
    Filter2D() : it_(n_){run();};
    ~Filter2D(){};
    void run();
    void callBack(const sensor_msgs::ImageConstPtr& msg);
};

void Filter2D::callBack(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Processing image ...");
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  img = cv_ptr -> image;

  

  cv_bridge::CvImage cv_img(std_msgs::Header(), "bgr8", img);
  sensor_msgs::ImagePtr msgs = cv_img.toImageMsg();
  it_pub_.publish(msgs);
}

void Filter2D::run()
{
  ROS_INFO("Fetching image ...");
  it_pub_ = it_.advertise("filter/filter2D/image", 1); 
  it_sub_ = it_.subscribe("pic_sim/image_raw", 10, &Filter2D::callBack, this);
}
*/

int main(int argc, char** argv)
{
  ROS_INFO("Initializing test ...");
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  

  Filter2D filter2D;
  ros::spinOnce();
  ros::loop_rate.sleep();
}
