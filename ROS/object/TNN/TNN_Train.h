#pragma once
#ifndef _TNN_TRAIN_H_
#define _TNN_TRAIN_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <vector>
#include "tiny-dnn/tiny_dnn/tiny_dnn.h"

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


#include "TNN_Network.h"
class TNN_Network;
/*
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
*/
class TNN_Train
{
  public:
    string working_directory = "/TNN_WORK_SPACE";
    string tnn_fileName;// = working_directory + "/mem";

    string data_dir_path;// = working_directory + "/MNIST/";
    string train_data_list;// = working_directory + "/data_list.txt";
	string network_cfg_file;
    int minibatch_size = 128;
    int num_epochs = 128;
    timer t;
    bool mnist_train = true;
    bool mnist_test = true;

  private:
    TNN_Network *nn_;
//    momentum optimizer_;
//    adagrad optimizer_;
    adam optimizer_;

    vector<vec_t>   train_image_;
    vector<label_t> train_label_;
    vector<vec_t>   test_image_;
    vector<label_t> test_label_;
    
    
    void progressShow(int& progress, int& total, int success, int total_num, int& freq);
    bool convert_image(Mat& imag, double scale, int w, int h, vector<vec_t>& data);

  public:
    TNN_Train();
    ~TNN_Train();
    bool construct_net();
    bool weight_save();
    bool weight_load();
    bool mnist_train_load();
    bool mnist_test_load();
    bool training_data_fetch(string& listFileName, double& scale, int& w, int& h, vector<vec_t>& data, vector<label_t>& label);
    void working_directory_set(string path);
    void run();
};

TNN_Train::TNN_Train()
{
  nn_ = new TNN_Network;
//  construct_net();
  working_directory_set(working_directory);
//  weight_load();
    
  train_image_.clear();
  train_label_.clear();
  test_image_.clear();
  test_label_.clear();
//  training_data_list_ = tnn_train_data_list;
//  mnist_train_load();
//  mnist_test_load();
}

TNN_Train::~TNN_Train()
{
  weight_save();
  delete nn_;
}

bool TNN_Train::construct_net()
{   
  if(nn_ != NULL)
    delete nn_;
  nn_ = new TNN_Network;
  if(nn_ -> construct_net(network_cfg_file))
  {
    nn_ -> show_net();
    return true;
  }
  else
  {
    return false;
  }
}


void TNN_Train::run()
{  
  // create callback
  int prog = 0;
  int total = num_epochs;
  tiny_dnn::result res;
  int success = 0;
  int success_total = 1;
  int count = 0;
  auto on_enumerate_epoch = [&]()
  {   
    res = nn_ -> nn.test(test_image_, test_label_);
    success = res.num_success;
    success_total = res.num_total;
    prog = prog + 1;
    weight_save();
  };

  auto on_enumerate_minibatch = [&]()
  {
    progressShow(prog , total, success, success_total, count);
    count += 1;
  };

  nn_ -> nn.train<mse>(optimizer_, train_image_, train_label_, minibatch_size, num_epochs
                     , on_enumerate_minibatch, on_enumerate_epoch);

  progressShow(prog , total, success, success_total, count);
}

bool TNN_Train::training_data_fetch(string& listFileName, double& scale, int& w, int& h, vector<vec_t>& data, vector<label_t>& label)
{
  ifstream listFile(listFileName.c_str());
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
      Mat trainImg = imread(p.string(), CV_LOAD_IMAGE_GRAYSCALE);
      if(convert_image(trainImg, scale, w, h, data))
      {
        label.push_back(catg);  
      }
    }
  }
  listFile.close();
  return true;
}

bool TNN_Train::convert_image(Mat& imag, double scale, int w, int h, vector<vec_t>& data)
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

void TNN_Train::progressShow(int& progress, int& total, int success, int total_num, int& freq)
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

bool TNN_Train::weight_save()
{
  return nn_ -> weight_save(tnn_fileName);
}

bool TNN_Train::weight_load()
{
  return nn_ -> weight_load(tnn_fileName);
}

bool TNN_Train::mnist_train_load()
{
  tiny_dnn::parse_mnist_labels(data_dir_path + "train-labels.idx1-ubyte", &train_label_);  
  tiny_dnn::parse_mnist_images(data_dir_path + "train-images.idx3-ubyte", &train_image_, -1.0, 1.0, 2, 2);  
  return true;
}

bool TNN_Train::mnist_test_load()
{
  tiny_dnn::parse_mnist_labels(data_dir_path + "t10k-labels.idx1-ubyte", &test_label_);  
  tiny_dnn::parse_mnist_images(data_dir_path + "t10k-images.idx3-ubyte", &test_image_, -1.0, 1.0, 2, 2);  
  return true;
}

void TNN_Train::working_directory_set(string path)
{
  boost::filesystem::path path_curt = current_path();
  working_directory = path_curt.string() + path;
  boost::filesystem::path dir(working_directory.c_str());
  if(!is_directory(dir))
    boost::filesystem::create_directories(dir);

  tnn_fileName = working_directory + "/mem";
  data_dir_path = working_directory + "/MNIST/";
  train_data_list = working_directory + "/data_list.txt";
  network_cfg_file = working_directory + "/network_model.cfg";
  
  ifstream inFile;
  ofstream outFile;
  inFile.open(train_data_list.c_str());
  if(inFile.fail())
  {
	outFile.open(train_data_list);
	outFile.close();
  }
  inFile.close();
  
  inFile.open(network_cfg_file.c_str());
  if(inFile.fail())
  {
	outFile.open(network_cfg_file);
	outFile.close();
  }
  inFile.close();
}

#endif
