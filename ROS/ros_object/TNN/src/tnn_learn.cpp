#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <tiny_dnn/tiny_dnn.h>
#include <vector>

using namespace std;
using namespace cv;
using namespace tiny_dnn;
using namespace tiny_dnn::activation;

#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
using namespace boost::filesystem;

//const int rate = 30;
const string working_directory = "/home/adel/Dropbox/ROS/TNN/";
const string nodeName = "TNN_Learn_Node";
const string tnn_fileName = working_directory + "mem";
const string tnn_learn_data_list = working_directory + "data_list.txt";

const string data_dir_path = "/home/adel/Dropbox/ROS/TNN/MNIST/";
const string tnn_topic_sub = "pic_sim/image_raw";
const string tnn_topic_pub = "tnn/image";
const string tnn_topic_rsub = "label";
const string tnn_topic_rpub = "tnn/label";



class TNN_Learn
{
  private:
    ros::NodeHandle n_;
    ros::Publisher r_pub_;
    ros::Subscriber r_sub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher it_pub_;
    image_transport::Subscriber it_sub_;

    network<sequential> nn_;
    momentum optimizer_;
//    adagrad optimizer_;
    string fileName_;
    string learning_data_list_;
    const static int imgInputSize_ = 32;
    const static int layerNum_ = 4;
    int convSize_[layerNum_];
    int convNum_[layerNum_];
    int poolSize_[layerNum_];
    const static int outputSize_ = 10;
    vector<vec_t> learn_image_;
    vector<label_t> learn_label_;
    vector<vec_t> test_image_;
    vector<label_t> test_label_;
    int minibatch_size_ = 10;
    int num_epochs_ = 20;
/*
    int max_training_cycle = 1;
    float training_threshold = 0.95;
*/
    bool mnist_learn_ = true;
    bool mnist_test_ = true;

    cv_bridge::CvImagePtr cv_ptr;
    Mat img;
    Mat img_gray;
    int scale_;
	
    timer t_;
	
    void construct_net(network<sequential>& nn_);
    void pubInit();
    void sub();
    void pubMat(Mat& img);
    void progressShow(int& progress, int& total, int success, int total_num, int& freq);
    void learnLabelCallBack(const std_msgs::UInt16::ConstPtr& msg);
    void learnImageCallBack(const sensor_msgs::ImageConstPtr& msg);
    bool convert_image(Mat& imag, double scale, int w, int h, vector<vec_t>& data);
    void mnist_learn_load();
    void mnist_test_load();
  public:
    TNN_Learn(ros::NodeHandle& n_);
    ~TNN_Learn();
    void run();
    bool weightSave();
    bool weightLoad();
    bool learning_data_fetch(string listFileName, double scale, int w, int h, vector<vec_t>& data, vector<label_t>& label);
    void train();
};

TNN_Learn::TNN_Learn(ros::NodeHandle& n) : n_(n), it_(n)
{
  learn_image_.clear();
  learn_label_.clear();
  fileName_ = tnn_fileName;
  learning_data_list_ = tnn_learn_data_list;
  scale_ = 1;
  convSize_[0] = 9; convSize_[1] = 5; convSize_[2] = 5; convSize_[3] = 3;
  convNum_[0] = 20; convNum_[1] = 40; convNum_[2] = 80; convNum_[3] = 160;
  poolSize_[0] = 1; poolSize_[1] = 2; poolSize_[2] = 1; poolSize_[3] = 2;
  
  pubInit();
  ROS_INFO("Constructing network ...");
  construct_net(nn_);
  for (int i = 0; i < nn_.depth(); i++) 
  {
    cout << "#Layer: " << i << "\n";
    cout << " Layer type:" << nn_[i]->layer_type() << "\n";
    cout << " Input:" << nn_[i]->in_size() << "(" << nn_[i]->in_shape() << ")\n";
    cout << " Output:" << nn_[i]->out_size() << "(" << nn_[i]->out_shape() << ")\n";
  }
  if(weightLoad())
  {
    ROS_INFO("Loading network model weight successfully !");
  }
  else 
  {
    ROS_WARN("Cannot loading network model weight !");
  }
  if(mnist_learn_)
  {
    ROS_INFO("MNIST training ...");
    ROS_INFO("Loading MNIST training source ...");
    mnist_learn_load();
    ROS_INFO("Loading MNIST testing source ...");
    mnist_test_load();
    ROS_INFO("Training ...");
    t_.restart();
    train();
    cout << endl;
    ROS_INFO("Trained : %f s elapsed", t_.elapsed());
    learn_image_.clear();
    learn_label_.clear();
  }
  else
  {
  if(learning_data_fetch(learning_data_list_, scale_, imgInputSize_, imgInputSize_, learn_image_, learn_label_))
  {
    ROS_INFO("Loading %s successfully !", learning_data_list_.c_str());
    cout << "Training images number : " << learn_image_.size() << "\n";
    cout << "Training labels number : " << learn_label_.size() << "\n";
    if(mnist_test_)
    {
      ROS_INFO("Loading MNIST testing source ...");
      mnist_test_load();
    }
    else
    {
      test_image_ = learn_image_;
      test_label_ = learn_label_;
    }
    ROS_INFO("Training ...");
    t_.restart();
    train();
    cout << endl;
    ROS_INFO("Trained : %f s elapsed", t_.elapsed());
  }
  else 
  {
    ROS_WARN("Cannot loading %s !", learning_data_list_.c_str());
    ofstream listFile(learning_data_list_);
    listFile.close();
  }
  }
  run();  
}

TNN_Learn::~TNN_Learn()
{
  ROS_INFO("Saving model weight in %s ...", fileName_.c_str());
  cout << "Saving model weight in " << fileName_ << " ...";
  if(weightSave())
  {
    cout << "ok!\n";
  }
  else
  {
    cout << "fail!\n";
  }
}

void TNN_Learn::construct_net(network<sequential>& nn_)
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

void TNN_Learn::learnLabelCallBack(const std_msgs::UInt16::ConstPtr& msg)
{
  ROS_INFO("Fetching learning label ...");
  if(msg == nullptr)
  {
    ROS_WARN("No learning label !");
  }
  else
  {
    ROS_INFO("Training label : %d", msg -> data);
    cout << msg -> data << "\n";
    learn_label_.push_back(msg -> data);
  }
//  r_pub_.publish(msg -> data);
}

void TNN_Learn::learnImageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Fetching learning image ...");
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  img = cv_ptr -> image;

//
  cvtColor(img, img_gray, CV_BGR2GRAY);
  
  if(!convert_image(img_gray, scale_, imgInputSize_, imgInputSize_, learn_image_))
  {
    ROS_WARN("Waiting for learning image ...");
    return;
  }
  r_sub_ = n_.subscribe(tnn_topic_rsub.c_str(), 1, &TNN_Learn::learnLabelCallBack, this);
  
  
  ROS_INFO("Training ...");
  t_.restart();
  train();
  cout << endl;
  ROS_INFO("Trained : %f s elapsed", t_.elapsed());   
//
  pubMat(img_gray);
}

void TNN_Learn::train()
{  
  // create callback
  int prog = 0;
  int total = num_epochs_;
  tiny_dnn::result res;
  int success = 0;
  int success_total = 1;
  int count = 0;
  auto on_enumerate_epoch = [&]()
  {	  
    res = nn_.test(test_image_, test_label_);
    success = res.num_success;
    success_total = res.num_total;
    prog = prog + 1;
    weightSave();
  };

  auto on_enumerate_minibatch = [&]()
  {
    progressShow(prog , total, success, success_total, count);
    count += 1;
  };

  nn_.train<mse>(optimizer_, learn_image_, learn_label_, minibatch_size_, num_epochs_
                 , on_enumerate_minibatch, on_enumerate_epoch);

  progressShow(prog , total, success, success_total, count);
//  auto img = nn_[0]->output_to_image();        
// save outputs of each layer  
/*
  for (size_t j = 0; j < nn_.depth(); j++)
  {  
      auto out_img = nn_[j]->output_to_image();  
      auto filename = working_directory + std::to_string(i) + "_layer_" + std::to_string(j) + ".png";  
      out_img.save(filename);  
  } 
*/
//  auto weight = nn_.at<tiny_dnn::convolutional_layer<tiny_dnn::tan_h>>(0).weight_to_image();  
//  weight.save(working_directory + "weight.png");
//  img.write("layer0.bmp");
}

void TNN_Learn::run()
{
  sub();
}

bool TNN_Learn::learning_data_fetch(string listFileName, double scale, int w, int h, vector<vec_t>& data, vector<label_t>& label)
{
  ifstream listFile(listFileName);
  if(listFile.fail())
  {
    return false;
  }
  string cont;
  string listPath;
  for(int catg = 1; getline(listFile, cont); catg++)
  {
    stringstream token(cont);
    token >> cont;
    token >> listPath;
	
    path dpath(listPath);
    BOOST_FOREACH(const path& p, make_pair(directory_iterator(dpath), directory_iterator())) 
    {
      if (is_directory(p)) continue;
	  cout << "Fetching category : " << catg << " = data : " << p.string() << "\n";
      Mat learnImg = imread(p.string(), CV_LOAD_IMAGE_GRAYSCALE);
      if(convert_image(learnImg, scale, w, h, data))
      {
        label.push_back(catg);	
      }
    }
  }
  listFile.close();
  return true;
}

bool TNN_Learn::convert_image(Mat& imag, double scale, int w, int h, vector<vec_t>& data)
{
  if(imag.data == nullptr)  return false;
  Mat_<uint8_t> resized;
  resize(imag, resized, cv::Size(w, h));
  
  vec_t d;
  transform(resized.begin(), resized.end(), std::back_inserter(d),
                   [=](uint8_t c) { return c * scale; });
  data.push_back(d);
  return true;
}

void TNN_Learn::progressShow(int& progress, int& total, int success, int total_num, int& freq)
{  
  int barWidth = 70;
  int pos = barWidth * progress / total;
  cout << "[";
  for (int i = 0; i < barWidth; ++i) 
  {
    if (i < pos) std::cout << "=";
    else if (i == pos) std::cout << ">";
    else cout << " ";
  }
  cout << "]";
  switch(freq % 4)
  {
    case 1 :
      cout << "\\";
      break;
    case 2 :
      cout << "|";
      break;
    case 3 :
      cout << "/";
      break;
    case 0 :
      cout << "-";
      freq = 0;
      break;
  }
  cout << " Progress: " << int(float(progress) / float(total) * 100.0) << "% Accuracy: " << int(float(success) / float(total_num) * 100.0) <<" %\r";
  cout.flush();
}

void TNN_Learn::sub()
{
//  r_sub_ = n_.subscribe(tnn_topic_rsub.c_str(), 1, &TNN_Learn::learnLabelCallBack, this);
  it_sub_ = it_.subscribe(tnn_topic_sub.c_str(), 1, &TNN_Learn::learnImageCallBack, this);
}

void TNN_Learn::pubInit()
{
  r_pub_ = n_.advertise<std_msgs::UInt16>(tnn_topic_rpub.c_str(), 1);
  it_pub_ = it_.advertise(tnn_topic_pub.c_str(), 1);
}

void TNN_Learn::pubMat(Mat& img)
{
  cv_bridge::CvImage cv_img(std_msgs::Header(), "mono8", img);
  sensor_msgs::ImagePtr msgs = cv_img.toImageMsg();
  it_pub_.publish(msgs);
}

bool TNN_Learn::weightSave()
{
  ofstream outFile(fileName_);
  if(!outFile.fail())
  {
    outFile << nn_;
    outFile.close();
    return true;
  }
  else 
  {
    return false;
  }
}

bool TNN_Learn::weightLoad()
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

void TNN_Learn::mnist_learn_load()
{
  tiny_dnn::parse_mnist_labels(data_dir_path + "train-labels.idx1-ubyte", &learn_label_);  
  tiny_dnn::parse_mnist_images(data_dir_path + "train-images.idx3-ubyte", &learn_image_, -1.0, 1.0, 2, 2);  
}

void TNN_Learn::mnist_test_load()
{
  tiny_dnn::parse_mnist_labels(data_dir_path + "t10k-labels.idx1-ubyte", &test_label_);  
  tiny_dnn::parse_mnist_images(data_dir_path + "t10k-images.idx3-ubyte", &test_image_, -1.0, 1.0, 2, 2);  
}

int main(int argc, char **argv) {
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());
  ros::NodeHandle n;
//  ros::Rate loop_rate(rate);
  
  ROS_INFO("Generating %s ...", nodeName.c_str());
  TNN_Learn tnn_l(n);
  /*
  while(n.ok())
  {
    tnn_l.train();
//    ros::spinOnce();
//    loop_rate.sleep();
    ros::spin();
  }
  */
}
