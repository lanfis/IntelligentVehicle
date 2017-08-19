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
#include "tiny-dnn/tiny_dnn/tiny_dnn.h"
#include "tiny-dnn/tiny_dnn/layers/layers.h"

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
    //string network_file_cfg_name = "network_model.cfg";
    int imgInputWidth = 32;
    int imgInputHeight = 32;
    int imgInputChannel = 1;
    /*
    int layerNum = 2;
    int convSize[layerNum];
    int convNum[layerNum];
    int poolSize[layerNum];
    */
    int imgOutputWidth = 10;
    int imgOutputHeight = 10;
//    int minibatch_size = 10;
//    int num_epochs = 20;
    
    network<sequential> nn;
    bool flag_load = false;
    bool flag_save = false;
    bool flag_network_construct = false;
        
  public:
    TNN_Network();
    ~TNN_Network();
    bool construct_net(string& tnn_fileName);
    void show_net();
    bool weight_save(string& tnn_fileName);
    bool weight_load(string& tnn_fileName);
    #ifdef DNN_USE_IMAGE_API
    void layer_activation_save(string& path);
    void layer_weight_save(string& path);
    #endif
};



TNN_Network::TNN_Network()
{
  //convSize[0] = 5; convSize[1] = 5;
  //convNum[0] = 10; convNum[1] = 20;
  //poolSize[0] = 2; poolSize[1] = 2;
}

TNN_Network::~TNN_Network()
{
}

bool TNN_Network::construct_net(string& tnn_fileName)
{   
  ifstream inFile(tnn_fileName.c_str());
  if(inFile.fail())
  {
      cout << "#Opening [" << tnn_fileName << "] fail !\n";
      flag_network_construct = false;
      return false;
  }
  int layer_width_in = imgInputWidth;
  int layer_height_in = imgInputHeight;
  int layer_channel_in = imgInputChannel;
  stringstream token;
  string str;
  string layer_name;
  for(int i = 0; getline(inFile, str); i++)
  {
    token.clear();
    token << str;
    token >> layer_name;
	cout << "#Constructing Layer " << i+1 << " : " << layer_name << endl;
    if(layer_name == "convolutional_layer")
    {
        int in_width = layer_width_in;//input image width
        int in_height = layer_height_in;//input image height
        int window_width = 5;//window_width(kernel) size of convolution 
        int window_height = 5;//window_height(kernel) size of convolution 
        int in_channels = layer_channel_in;//input image channels (grayscale=1, rgb=3) 
        int out_channels = 3;//soutput image channels 
		string pad = "valid";//rounding strategy
		/*– valid: use valid pixels of input only. output-size = (in-width - window_width + 1) * (in-height - window_height + 1) * out_channels
  		  – same: add zero-padding to keep same width/height. output-size = in-width * in-height * out_channels */
		bool has_bias = true;//whether to add a bias vector to the ﬁlter outputs 
		int  w_stride = 1;//specify the horizontal interval at which to apply the ﬁlters to the input 
		int  h_stride = 1;//specify the horizontal interval at which to apply the ﬁlters to the input 
        while(token >> str)
        {
            if(str == "in_width") token >> in_width;
            if(str == "in_height") token >> in_height;
            if(str == "window_width") token >> window_width;
            if(str == "window_height") token >> window_height;
            if(str == "in_channels") token >> in_channels;
            if(str == "out_channels") token >> out_channels;
            if(str == "padding") token >> pad;
            if(str == "has_bias")
			{
				token >> str;
                if(str == "false")
                    has_bias = false;
                else
                    has_bias = true;
			}
            if(str == "w_stride") token >> w_stride;
            if(str == "h_stride") token >> h_stride;
        }
		if(pad == "same")
		{
          nn << convolutional_layer(in_width, in_height, window_width, window_height, in_channels, out_channels, padding::same, has_bias, w_stride, h_stride, core::default_engine());
          layer_width_in = in_width;
          layer_height_in = in_height;
		}
	    else
		{
          nn << convolutional_layer(in_width, in_height, window_width, window_height, in_channels, out_channels, padding::valid, has_bias, w_stride, h_stride, core::default_engine());
          layer_width_in = in_width - window_width + 1;
          layer_height_in = in_height - window_height + 1;
		}
        layer_channel_in = out_channels;
    }
    if(layer_name == "average_pooling_layer")
    {
        int in_width = layer_width_in;//input image width
        int in_height = layer_height_in;//input image height
        int in_channels = layer_channel_in;//sthe number of input image channels(depth)
        int pool_size_x = 2;//factor by which to downscale
        int pool_size_y = 2;//factor by which to downscale
		int stride_x = 1;//interval at which to apply the ﬁlters to the input 
		int stride_y = 1;//interval at which to apply the ﬁlters to the input 
		string pad = "valid";//padding mode(same/valid)
        while(token >> str)
        {
            if(str == "in_width") token >> in_width;
            if(str == "in_height") token >> in_height;
            if(str == "in_channels") token >> in_channels;
            if(str == "pool_size_x") token >> pool_size_x;
            if(str == "pool_size_y") token >> pool_size_y;
            if(str == "stride_x") token >> stride_x;
            if(str == "stride_y") token >> stride_y;
            if(str == "padding") token >> pad;
        }
        if(in_width % pool_size_x != 0)
        {
            cout << "#average_pooling_layer " << i+1 << " : in_width/pool_size_x = " << in_width << "/" << pool_size_x << " cannot divisible !\n";
        }
        if(in_height % pool_size_y != 0)
        {
            cout << "#average_pooling_layer " << i+1 << " : in_height/pool_size_y = " << in_height << "/" << pool_size_y << " cannot divisible !\n";
        }
		if(pad == "same")
		{
          nn << average_pooling_layer(in_width, in_height, in_channels, pool_size_x, pool_size_y, stride_x, stride_y, padding::same);
          layer_width_in = in_width;
          layer_height_in = in_height;
		}
		else
		{
          nn << average_pooling_layer(in_width, in_height, in_channels, pool_size_x, pool_size_y, stride_x, stride_y, padding::valid);
          layer_width_in = in_width/pool_size_x;
          layer_height_in = in_height/pool_size_y;
		}
    }
    if(layer_name == "fully_connected_layer")
    {
        int in_dim = layer_channel_in;//number of elements of the input
        int out_dim = 120;//number of elements of the output 
        bool has_bias = true;//whether to include additional bias to the layer 
        while(token >> str)
        {
            if(str == "in_dim") token >> in_dim;
            if(str == "out_dim") token >> out_dim;
            if(str == "has_bias")
            {
                token >> str;
                if(str == "false")
                    has_bias = false;
                else
                    has_bias = true;
            }
        }
        nn << fully_connected_layer(in_dim, out_dim, has_bias);
        layer_channel_in = out_dim;
    } 
    if(layer_name == "max_pooling_layer")
    {
        int in_width = layer_width_in;//input image width
        int in_height = layer_height_in;//input image height
        int in_channels = layer_channel_in;//sthe number of input image channels(depth)
        int pool_size_x = 2;//factor by which to downscale
        int pool_size_y = 2;//factor by which to downscale
		int stride_x = 2;//interval at which to apply the ﬁlters to the input 
		int stride_y = 2;//interval at which to apply the ﬁlters to the input 
		string pad = "valid";//padding mode(same/valid)
        while(token >> str)
        {
            if(str == "in_width") token >> in_width;
            if(str == "in_height") token >> in_height;
            if(str == "in_channels") token >> in_channels;
            if(str == "pool_size_x") token >> pool_size_x;
            if(str == "pool_size_y") token >> pool_size_y;
            if(str == "stride_x") token >> stride_x;
            if(str == "stride_y") token >> stride_y;
            if(str == "padding") token >> pad;
        }
        if(in_width % pool_size_x != 0)
        {
            cout << "#max_pooling_layer " << i+1 << " : in_width/pool_size_x = " << in_width << "/" << pool_size_x << " cannot divisible !\n";
        }
        if(in_height % pool_size_y != 0)
        {
            cout << "#max_pooling_layer " << i+1 << " : in_height/pool_size_y = " << in_height << "/" << pool_size_y << " cannot divisible !\n";
        }
		if(pad == "same")
		{
          nn << max_pooling_layer(in_width, in_height, in_channels, pool_size_x, pool_size_y, stride_x, stride_y, padding::same, core::default_engine());
          layer_width_in = in_width;
          layer_height_in = in_height;
		}
		else
		{
          nn << max_pooling_layer(in_width, in_height, in_channels, pool_size_x, pool_size_y, stride_x, stride_y, padding::valid, core::default_engine());
          layer_width_in = in_width/pool_size_x;
          layer_height_in = in_height/pool_size_y;
		}
    }
    if(layer_name == "dropout_layer")
    {
        int in_dim = layer_channel_in;//number of elements of the input 
        float dropout_rate = 0.5;//input image height
        while(token >> str)
        {
            if(str == "in_dim") token >> in_dim;
            if(str == "dropout_rate") token >>  dropout_rate;
        }
        nn << dropout_layer(in_dim,  dropout_rate);
    }
	
    if(layer_name == "elu_layer")
        nn << elu_layer();
    if(layer_name == "leaky_relu_layer")
        nn << leaky_relu_layer();
    if(layer_name == "relu_layer")
        nn << relu_layer();
    if(layer_name == "selu_layer")
        nn << selu_layer();
    if(layer_name == "sigmoid_layer")
        nn << sigmoid_layer();
    if(layer_name == "softmax_layer")
        nn << softmax_layer();
    if(layer_name == "softplus_layer")
        nn << softplus_layer();
    if(layer_name == "softsign_layer")
        nn << softsign_layer();
    if(layer_name == "tanh_layer")
        nn << tanh_layer();
    if(layer_name == "tanh_p1m2_layer")
        nn << tanh_p1m2_layer();
    /*
    
    convOut = convNum[i];
    nn << convolutional_layer(convIn, convIn, convSize[i], convSize[i], layNum, convOut) << relu_layer();
    poolIn = convIn - convSize[i] + 1;
    nn << average_pooling_layer(poolIn, poolIn, convOut, poolSize[i]) << tanh_layer();
    convIn = poolIn / poolSize[i];
    layNum = convOut;
    
    
  nn << fully_connected_layer(convIn * convIn * layNum, convIn * outputSize)
     << fully_connected_layer(convIn * outputSize, outputSize)
     << softmax_layer(outputSize)
  ; */
  }
  inFile.close();
  flag_network_construct = true;
  return true;
}


void TNN_Network::show_net()
{
  for (int i = 0; i < nn.depth() && flag_network_construct; i++) 
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
  /*string arc = tnn_fileName + ".json";
  string wei = tnn_fileName;
  cout << "Architecture : " << arc << " saving ...\n";
  nn.save(arc.c_str(), content_type::model, file_format::json);
  cout << "Model and Weight : " << wei << " saving ...\n";
  nn.load(wei.c_str(), content_type::weights_and_model, file_format::binary);
  return true;*/
  
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

#ifdef DNN_USE_IMAGE_API
void TNN_Network::layer_activation_save(string& path)
{     
// save outputs of each layer  
  for (size_t i = 0; i < nn.depth(); i++) 
  {
      auto out_img = nn[i]->output_to_image();// visualize activations of recent input 
      auto filename = path + "/layer_" + std::to_string(i) + ".png";
      out_img.save(filename);
  } 
}

void TNN_Network::layer_weight_save(string& path)
{     
  // save filter shape of first convolutional layer
  for (size_t i = 0; i < nn.depth(); i++) 
  {
      auto weight = nn.at<convolutional_layer>(i).weight_to_image();
      auto filename = path + "/layer_weights_" + std::to_string(i) + ".png";
      weight.save(filename);
  }
}
#endif

#endif
