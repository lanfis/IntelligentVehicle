#pragma once
#ifndef _TNN_NETWORK_H_
#define _TNN_NETWORK_H_

#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <vector>
#include "TNN/tiny-dnn/tiny_dnn/tiny_dnn.h"
#include "TNN/tiny-dnn/tiny_dnn/layers/layers.h"

using namespace std;
using namespace cv;
using namespace tiny_dnn;
using namespace tiny_dnn::activation;
using namespace tiny_dnn::layers;

using namespace boost::filesystem;


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


class TNN_Network
{
  public:
    const static int imgInputSize = 32;
    const static int layerNum = 2;
    int convSize[layerNum];
    int convNum[layerNum];
    int poolSize[layerNum];
    const static int outputSize = 10;
//    int minibatch_size = 10;
//    int num_epochs = 20;
    
    network<sequential> nn;
    bool flag_load = false;
    bool flag_save = false;
        
  public:
    TNN_Network();
    ~TNN_Network();
    void construct_net();
    void show_net();
    bool weight_save(string& tnn_fileName);
    bool weight_load(string& tnn_fileName);
//    void layer_save(string& path);
};



TNN_Network::TNN_Network()
{
  convSize[0] = 5; convSize[1] = 5;
  convNum[0] = 10; convNum[1] = 20;
  poolSize[0] = 2; poolSize[1] = 2;
}

TNN_Network::~TNN_Network()
{
}

void TNN_Network::construct_net()
{   
  int convIn = imgInputSize;
  int convOut;
  int poolIn;
  int layNum = 1;
  for(int i = 0; i < layerNum; i++)
  {
    convOut = convNum[i];
    nn << convolutional_layer(convIn, convIn, convSize[i], convSize[i], layNum, convOut) << relu_layer();
    poolIn = convIn - convSize[i] + 1;
    nn << max_pooling_layer(poolIn, poolIn, convOut, poolSize[i]) << tanh_layer();
    convIn = poolIn / poolSize[i];
    layNum = convOut;
  }
  nn << fully_connected_layer(convIn * convIn * layNum, convIn * outputSize)
     << fully_connected_layer(convIn * outputSize, outputSize);
  ; 
  /*
  nn << fully_connected_layer<identity>(convIn * convIn * layNum, convIn * outputSize)
     << fully_connected_layer<identity>(convIn * outputSize, outputSize);
  ; 
  */
}


void TNN_Network::show_net()
{
  for (int i = 0; i < nn.depth(); i++) 
  {
    cout << "#Layer: " << i << "\n";
    cout << " Layer type:" << nn[i]->layer_type() << "\n";
    cout << " Input:" << nn[i]->in_size() << "(" << nn[i]->in_shape() << ")\n";
    cout << " Output:" << nn[i]->out_size() << "(" << nn[i]->out_shape() << ")\n";
  }
}

bool TNN_Network::weight_save(string& tnn_fileName)
{
  ofstream outFile(tnn_fileName);
  if(!outFile.fail())
  {
    outFile << nn;
    outFile.close();
    flag_save = true;
    return true;
  }
  else 
  {
    flag_save = false;
    return false;
  }
}

bool TNN_Network::weight_load(string& tnn_fileName)
{
  ifstream inFile(tnn_fileName);
  if(!inFile.fail())
  {
    inFile >> nn;
    inFile.close();
    flag_load = true;
    return true;
  }
  else 
  {
    flag_load = false;
    return false;
  }
}
/*
void TNN_Network::layer_save(string& path)
{     
// save outputs of each layer  
  for (size_t i = 0; i < nn.depth(); i++) 
  {
      auto out_img = nn[i]->output_to_image();
      auto filename = path + "layer_" + std::to_string(i) + ".png";
      out_img.save(filename);
  }
  // save filter shape of first convolutional layer
  {
      auto weight = nn.at<convolutional_layer<tan_h>>(0).weight_to_image();
      auto filename = path + "weights.png";
      weight.save(filename);
  }
}
*/
#endif
