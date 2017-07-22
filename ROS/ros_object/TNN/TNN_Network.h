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
#include "tiny_dnn/tiny_dnn.h"
#include "tiny_dnn/layers/layers.h"

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
    const static int layerNum = 4;
    int convSize[layerNum];
    int convNum[layerNum];
    int poolSize[layerNum];
    const static int outputSize = 10;
    int minibatch_size = 10;
    int num_epochs = 20;
    
    network<sequential> nn;
    
    
  public:
    TNN_Network();
    ~TNN_Network();
    void construct_net();
    void show_net();
    bool weight_save(string& tnn_fileName);
    bool weight_load(string& tnn_fileName);
};



TNN_Network::TNN_Network()
{
  convSize[0] = 9; convSize[1] = 5; convSize[2] = 5; convSize[3] = 3;
  convNum[0] = 20; convNum[1] = 40; convNum[2] = 80; convNum[3] = 160;
  poolSize[0] = 1; poolSize[1] = 2; poolSize[2] = 1; poolSize[3] = 2;
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
    nn << convolutional_layer<relu>(convIn, convIn, convSize[i], convSize[i], layNum, convOut);
    poolIn = convIn - convSize[i] + 1;
    nn << max_pooling_layer<identity>(poolIn, poolIn, convOut, poolSize[i]);
    convIn = poolIn / poolSize[i];
    layNum = convOut;
  }
  nn << fully_connected_layer<identity>(convIn * convIn * layNum, convIn * outputSize)
     << fully_connected_layer<identity>(convIn * outputSize, outputSize);
  ; 
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
    return true;
  }
  else 
  {
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
    return true;
  }
  else 
  {
    return false;
  }
}

#endif