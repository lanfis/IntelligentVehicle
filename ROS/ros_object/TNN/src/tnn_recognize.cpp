#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <tiny_dnn/tiny_dnn.h>
#include <tiny_dnn/layers/layers.h>
#include <vector>

using namespace std;
using namespace cv;
using namespace tiny_dnn;
using namespace tiny_dnn::activation;
using namespace tiny_dnn::layers;

#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
using namespace boost::filesystem;

const string working_directory = "/home/adel/Dropbox/ros_kinetic/TNN/";
const string nodeName = "TNN_Recognize_Node";
//const int rate = 30;
const string tnn_topic_sub = "pic_sim/image_raw";
const string tnn_topic_pub = "tnn/image";
const string tnn_topic_rsub = "label";
const string tnn_topic_rpub = "tnn/label";
const string tnn_fileName = working_directory + "mem_lenet5";


// rescale output to 0-100
template <typename Activation>
double rescale(double x) {
    Activation a;
    return 100.0 * (x - a.scale().first) / (a.scale().second - a.scale().first);
}

// convert tiny_cnn::image to cv::Mat and resize
template <typename image>
cv::Mat image2mat(image& img) {
    cv::Mat ori(img.height(), img.width(), CV_8U, &img.at(0, 0));
    cv::Mat resized;
    cv::resize(ori, resized, cv::Size(), 3, 3, cv::INTER_AREA);
    return resized;
}


class TNN_Recognize
{
  private:
    ros::NodeHandle n_;
    ros::Publisher r_pub_;
    ros::Subscriber r_sub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher it_pub_;
    image_transport::Subscriber it_sub_;

    network<sequential> nn_;
    adagrad optimizer_;
    string fileName_;
    const static int imgInputSize_ = 32;
    const static int layerNum_ = 4;
    int convSize_[layerNum_];
    int convNum_[layerNum_];
    int poolSize_[layerNum_];
    const static int outputSize_ = 10;
  
    cv_bridge::CvImagePtr cv_ptr;
    Mat img;
    double minv;
    double maxv;
    timer t;
    vec_t recognize_image_;
    label_t recognize_label_;
    int minibatch_size_ = 10;
    int num_epochs_ = 20;
	
    void construct_net(network<sequential>& nn_);
    void pub();
    void sub();
    void recognizeLabelCallBack(const std_msgs::UInt16::ConstPtr& msg);
    void recognizeImageCallBack(const sensor_msgs::ImageConstPtr& msg);
    bool convert_image(Mat& img, double minv, double maxv, int w, int h, vec_t& data);
  public:
    TNN_Recognize(ros::NodeHandle& n_);
    ~TNN_Recognize();
    void run();
    bool load();
};

TNN_Recognize::TNN_Recognize(ros::NodeHandle& n) : n_(n), it_(n)
{
  fileName_ = tnn_fileName;
  minv = -1;
  maxv = 1;
  convSize_[0] = 9; convSize_[1] = 5; convSize_[2] = 5; convSize_[3] = 3;
  convNum_[0] = 20; convNum_[1] = 40; convNum_[2] = 80; convNum_[3] = 160;
  poolSize_[0] = 1; poolSize_[1] = 2; poolSize_[2] = 1; poolSize_[3] = 2;
  
  pub();
  ROS_INFO("Constructing network ...");
  construct_net(nn_);
  for (int i = 0; i < nn_.depth(); i++) 
  {
    cout << "#Layer: " << i << "\n";
    cout << " Layer type:" << nn_[i]->layer_type() << "\n";
    cout << " Input:" << nn_[i]->in_size() << "(" << nn_[i]->in_shape() << ")\n";
    cout << " Output:" << nn_[i]->out_size() << "(" << nn_[i]->out_shape() << ")\n";
  }
  if(load())
  {
    ROS_INFO("Loading network model weight successfully !");
  }
  else 
  {
    ROS_WARN("Cannot loading network model weight !");
  }
  run();  
}

TNN_Recognize::~TNN_Recognize()
{
}

void TNN_Recognize::construct_net(network<sequential>& nn_)
{	
  int convIn = imgInputSize_;
  int convOut;
  int poolIn;
  int layNum = 1;
  for(int i = 0; i < layerNum_; i++)
  {
    convOut = convNum_[i];
    nn_ << convolutional_layer<relu>(convIn, convIn, convSize_[i], convSize_[i], layNum, convOut);
    poolIn = convIn - convSize_[i] + 1;
    nn_ << max_pooling_layer<identity>(poolIn, poolIn, convOut, poolSize_[i]);
    convIn = poolIn / poolSize_[i];
    layNum = convOut;
  }
  nn_ << fully_connected_layer<identity>(convIn * convIn * layNum, convIn * outputSize_)
      << fully_connected_layer<identity>(convIn * outputSize_, outputSize_);
  ; 
}
/*
void TNN_Recognize::recognizeLabelCallBack(const std_msgs::UInt16::ConstPtr& msg)
{
  ROS_INFO("Fetching training label ...");
  if(msg == nullptr)
  {
    ROS_WARN("No training label !");
    train_label_ = {0};
  }
  else
  {
    ROS_INFO("Training label : %d", msg -> data);
    cout << msg -> data << "\n";
    train_label_ = {msg -> data};
  }
//  r_pub_.publish(msg -> data);
}
*/
void TNN_Recognize::recognizeImageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Fetching training image ...");
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  img = cv_ptr -> image;

//
  Mat img_gray;
  cvtColor(img, img_gray, CV_BGR2GRAY);
  recognize_image_.clear();
  if(!convert_image(img_gray, minv, maxv, imgInputSize_, imgInputSize_, recognize_image_))
  {
    ROS_WARN("Waiting for training image ...");
    return;
  }
  
  
  ROS_INFO("Recognizing ...");
  t.restart();
  vec_t res = nn_.predict(recognize_image_);
  vector<pair<double, int> > scores;

  
  // sort & print top-3
  for (int i = 0; i < res.size(); i++)
      scores.emplace_back(rescale<tan_h>(res[i]), i);

  sort(scores.begin(), scores.end(), greater<pair<double, int>>());

  for (int i = 0; i < 3; i++)
      cout << scores[i].second << "," << scores[i].first << endl;

  // save outputs of each layer
/*
  for (size_t i = 0; i < nn_.depth(); i++) {
      auto out_img = nn_[i]->output_to_image();
      auto filename = "layer_" + std::to_string(i) + ".png";
      out_img.save(filename);
  }
  // save filter shape of first convolutional layer
  {
      auto weight = nn_.at<convolutional_layer<tan_h>>(0).weight_to_image();
      auto filename = "weights.png";
      weight.save(filename);
  }
*/

  ROS_INFO("Recognized : %f s elapsed", t.elapsed());
//

  cv_bridge::CvImage cv_img(std_msgs::Header(), "mono8", img_gray);
  sensor_msgs::ImagePtr msgs = cv_img.toImageMsg();
  it_pub_.publish(msgs);
}

void TNN_Recognize::run()
{
  sub();
}
/*
bool TNN_Recognize::image_adjust(Mat& src, int w, int h)
{
  if(img.data == nullptr)  return false;
  Mat img_resized;
  resize(src, img_resized, cv::Size(w, h));
  
}
*/
bool TNN_Recognize::convert_image(Mat& img, double minv, double maxv, int w, int h, vec_t& data)
{
  if(img.data == nullptr)  return false;
  Mat_<uint8_t> resized;
  resize(img, resized, cv::Size(w, h));
  //double thresh = 100;
  //cv::threshold( resized, resized, thresh, 255, THRESH_BINARY );
  img = resized;
  // mnist dataset is "white on black", so negate required
  std::transform(resized.begin(), resized.end(), std::back_inserter(data),
      [=](uint8_t c) { return (255 - c) * (maxv - minv) / 255.0 + minv; });
  
  return true;
}

void TNN_Recognize::sub()
{
//  r_sub_ = n_.subscribe(tnn_topic_rsub.c_str(), 1, &TNN_Recognize::recognizeLabelCallBack, this);
  it_sub_ = it_.subscribe(tnn_topic_sub.c_str(), 1, &TNN_Recognize::recognizeImageCallBack, this);
}

void TNN_Recognize::pub()
{
  r_pub_ = n_.advertise<std_msgs::UInt16>(tnn_topic_rpub.c_str(), 1);
  it_pub_ = it_.advertise(tnn_topic_pub.c_str(), 1);
}

bool TNN_Recognize::load()
{
  ifstream inFile(fileName_);
  if(!inFile.fail())
  {
    inFile >> nn_;
    inFile.close();
    return true;
  }
  else 
  {
    return false;
  }
}

int main(int argc, char **argv) {
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());
  ros::NodeHandle n;
//  ros::Rate loop_rate(rate);
  
  ROS_INFO("Generating %s ...", nodeName.c_str());
  TNN_Recognize tnn_r(n);
  while(n.ok())
  {
    tnn_r.run();
//    ros::spinOnce();
//    loop_rate.sleep();
    ros::spin();
  }
}
