#pragma once
#ifndef _TNN_RECOGNIZE_H_
#define _TNN_RECOGNIZE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <vector>
#include "TNN/tiny-dnn/tiny_dnn/tiny_dnn.h"
#include "TNN/tiny-dnn/tiny_dnn/layers/layers.h"

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

class TNN_Recognize
{
  public:
    string working_directory = "/TNN_WORK_SPACE";
    string tnn_fileName;// = working_directory + "/mem";

    timer t;
    int label;
    double accuracy;

  private:
    TNN_Network *nn_;

    double minv;
    double maxv;
    vec_t recognize_image_;
    label_t recognize_label_;
    
    Mat img_gray;
    bool convert_image(Mat& img, double& minv, double& maxv, int w, int h, vec_t& data);
    
  public:
    TNN_Recognize();
    ~TNN_Recognize();
    void construct_net();
    bool weight_load();
    bool run(Mat& img);
    void working_directory_set(string path);
};

TNN_Recognize::TNN_Recognize()
{
    nn_ = new TNN_Network;
    minv = -1;
    maxv = 1;
//    construct_net();
    working_directory_set(working_directory);
//    weight_load();
}

TNN_Recognize::~TNN_Recognize()
{
    delete nn_;
}

bool TNN_Recognize::weight_load()
{
  return nn_ -> weight_load(tnn_fileName);
}

void TNN_Recognize::construct_net()
{
    delete nn_;
    nn_ = new TNN_Network;
    nn_ -> construct_net();
    nn_ -> show_net();
}

bool TNN_Recognize::run(Mat& img)
{
  if(!nn_ -> flag_load)
    return false;
  cvtColor(img, img_gray, CV_BGR2GRAY);
  recognize_image_.clear();
  if(!convert_image(img_gray, minv, maxv, nn_ -> imgInputSize, nn_ -> imgInputSize, recognize_image_))
  {
    cout << "[TNN_Recognize] cannot convert image !\n";
    return false;
  }
  
  t.restart();
  auto res = nn_ -> nn.predict(recognize_image_);//vec_t
  vector<pair<double, int> > scores;

  // sort & print top-3
  for (int i = 0; i < res.size(); i++)
      scores.emplace_back(rescale<tanh_layer>(res[i]), i);

  sort(scores.begin(), scores.end(), greater<pair<double, int>>());
  label = scores[0].second;
  accuracy = scores[0].first;

  for (int i = 0; i < 3; i++)
      cout << scores[i].second << "," << scores[i].first << endl;
  res.clear();
  scores.clear();
  return true;
}


bool TNN_Recognize::convert_image(Mat& img, double& minv, double& maxv, int w, int h, vec_t& data)
{
  if(img.data == nullptr)  return false;
  Mat_<uint8_t> resized;
  resize(img, resized, cv::Size(w, h));
  //double thresh = 100;
  //cv::threshold( resized, resized, thresh, 255, THRESH_BINARY );
  //img = resized;
  // mnist dataset is "white on black", so negate required
  std::transform(resized.begin(), resized.end(), std::back_inserter(data),
      [=](uint8_t c) { return (255 - c) * (maxv - minv) / 255.0 + minv; });
  
  return true;
}


void TNN_Recognize::working_directory_set(string path)
{
  boost::filesystem::path path_curt = current_path();
  working_directory = path_curt.string() + path;
  boost::filesystem::path dir(working_directory.c_str());
  if(!is_directory(dir))
    boost::filesystem::create_directories(dir);

  tnn_fileName = working_directory + "/mem";
}


#endif
