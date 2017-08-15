#pragma once
#ifndef _MOTION_KNN_H_
#define _MOTION_KNN_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"

using namespace std;
using namespace cv;

class MOTION_KNN
{
    public:
      MOTION_KNN();
      ~MOTION_KNN();
      void run(Mat& image, Mat& fgMask, vector<Rect>& box);
	  void init();

    public:
      int history;//Length of the history. 
      float dist2Threshold;//Threshold on the squared distance between the pixel and the sample to decide whether a pixel is close to that sample. This parameter does not affect the background update. 
	  bool detectShadows;//If true, the algorithm will detect shadows and mark them. It decreases the speed a bit, so if you do not need this feature, set the parameter to false. 
      float min_box_length_ratio;
      float max_box_length_ratio;
      float learning_rate;
      bool auto_adjust;
    
    private:
      int min_box_length;
      int max_box_length;
      int num_detect_;  
	  float line_ratio = 0.20;
    
    private:
	  Mat fgMask;
	  Ptr<BackgroundSubtractor> pKNN;
	  
      Mat img_past;
      Mat img_current;
      void detect(Mat& fgMask, vector<Rect>& boundBox);
      void adjust();
};

MOTION_KNN::MOTION_KNN()
{
    history = 500;
    dist2Threshold = 400.0;
	detectShadows = false;
    min_box_length_ratio = 15.0/640.0;
    max_box_length_ratio = 400.0/640.0;
    learning_rate = 0.01;
    auto_adjust = true;
}

MOTION_KNN::~MOTION_KNN()
{}

void MOTION_KNN::init()
{
	pKNN = createBackgroundSubtractorKNN(history, dist2Threshold, detectShadows); 
	learning_rate = 0.1;
}

void MOTION_KNN::run(Mat& image, Mat& fgMask, vector<Rect>& box)
{
    min_box_length = int(min_box_length_ratio*float(image.cols));
    max_box_length = int(max_box_length_ratio*float(image.cols));
	pKNN -> apply(image, fgMask, learning_rate);
    detect(fgMask, box);
    adjust();
}

void MOTION_KNN::detect(Mat& fgMask, vector<Rect>& boundBox)
{
  vector<vector<cv::Point> > contours;
  vector<Vec4i> hierarchy;
  //cv::dilate(fgMask, fgMask, Mat(), Point(-1, -1), 2);
  //cv::erode(fgMask, fgMask, Mat(), Point(-1, -1), 4);
  //cv::dilate(fgMask, fgMask, Mat(), Point(-1, -1), 2);
  /// Find contours
  cv::findContours( fgMask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Approximate contours to polygons + get bounding rects and circles
  vector<Rect> boundRect( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
  {
    if(max_box_length*max_box_length > contourArea(contours[i]) && contourArea(contours[i]) > min_box_length*min_box_length)
    {
      Rect bound = boundingRect( Mat(contours[i]) );
      if(bound.width < min_box_length || bound.height < min_box_length || bound.width > max_box_length || bound.height > max_box_length) continue;
	  if(float(bound.width) < line_ratio * float(bound.height) || float(bound.height) < line_ratio * float(bound.width)) continue;
      //boundRect[i] = bound;//boundingRect( Mat(contours[i]) );
      boundBox.push_back(bound);
      //rectangle( img_current, bound.tl(), bound.br(), cv::Scalar(0, 128, 0), 2, 8, 0 );
    }
  }
  num_detect_ = boundBox.size();
}

void MOTION_KNN::adjust()
{
  if(!auto_adjust) return;
  if(num_detect_ != 0)
  {
    learning_rate = (learning_rate - 1.0/history < 0)? 1.0/history : learning_rate - 10.0/history;
  }
  else
  {
	learning_rate = (learning_rate + 1.0/history > 0.2)? 0.2 : learning_rate + 1.0/history;
  }
}

#endif
