#pragma once
#ifndef _MOTION_H_
#define _MOTION_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"

using namespace std;
using namespace cv;

class MOTION
{
    public:
      MOTION();
      ~MOTION();
      void run(Mat& image, vector<Rect>& box);
      void init(Mat& img);

    public:
      int delay_count;
      int thresh;
      float min_box_length_ratio;
      float max_box_length_ratio;
      int bg_filter_ksize;
      float bg_filter_sigma;
      int num_detect;
      int num_detect_tor;
      int num_detect_diff;
      bool auto_adjust;
    
    private:
      vector<Rect> box_past_;
      int min_box_length;
      int max_box_length;
      int num_detect_;  
      int num_detect_past_;  
      int delay_count_; 
    
    private:
      Mat img_past;
      Mat img_current;
      void detect(vector<Rect>& boundBox);
      void adjust();
};

MOTION::MOTION()
{
    delay_count = 1;
    thresh = 20;
    min_box_length_ratio = 15.0/640.0;
    max_box_length_ratio = 400.0/640.0;
    bg_filter_ksize = 3;
    bg_filter_sigma = 1;
    num_detect = 3;
    num_detect_tor = num_detect;
    num_detect_diff = 5;
    delay_count_ = delay_count;
    auto_adjust = true;
}

MOTION::~MOTION()
{}

void MOTION::init(Mat& img)
{
  if(img_past.empty())
  {
    img.copyTo(img_past);
    img.copyTo(img_current);
  }
  min_box_length = int(min_box_length_ratio*float(img.cols));
  max_box_length = int(max_box_length_ratio*float(img.cols));
}

void MOTION::run(Mat& img, vector<Rect>& box)
{/*
  if(delay_count_ > 0)
  {
    delay_count_ -= 1;
    box = this -> box_;
    return;
  }
  else delay_count_ = delay_count;
  */
  if(img_past.empty()) init(img);
  if(img_current.empty()) return;
  
  img_current.copyTo(img_past);
  img.copyTo(img_current);
  
  //img_past = img_current;
  //img_current = img;
  detect(box);
  adjust();
}

void MOTION::detect(vector<Rect>& boundBox)
{
  cv::Mat img_diff;// = cv::Mat::zeros(img_current.size(), CV_8UC3);
  cv::Mat img_past_gray;// = cv::Mat::zeros(img_current.size(), CV_8UC3);
  cv::Mat img_current_gray;// = cv::Mat::zeros(img_current.size(), CV_8UC3);
  cv::cvtColor(img_current, img_current_gray, CV_BGR2GRAY);
  cv::cvtColor(img_past, img_past_gray, CV_BGR2GRAY);
  cv::absdiff(img_current_gray, img_past_gray, img_diff);
//  cv::blur(img_diff, img_diff, Size(3, 3));
  GaussianBlur(img_diff, img_diff, Size(bg_filter_ksize, bg_filter_ksize), bg_filter_sigma);

  cv::Mat img_thresh = cv::Mat::zeros(img_diff.size(), CV_8UC1);
  vector<vector<cv::Point> > contours;
  vector<Vec4i> hierarchy;
/// Detect edges using Threshold
  cv::threshold( img_diff, img_thresh, thresh, 255, THRESH_BINARY );
  cv::dilate(img_thresh, img_thresh, Mat(), Point(-1, -1), 2);
  /// Find contours
  cv::findContours( img_thresh, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Approximate contours to polygons + get bounding rects and circles
  vector<Rect> boundRect( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
  {
    if(max_box_length*max_box_length > contourArea(contours[i]) && contourArea(contours[i]) > min_box_length*min_box_length)
    {
      Rect bound = boundingRect( Mat(contours[i]) );
      if(bound.width < min_box_length || bound.height < min_box_length || bound.width > max_box_length || bound.height > max_box_length) continue;
      //boundRect[i] = bound;//boundingRect( Mat(contours[i]) );
      boundBox.push_back(bound);
      //rectangle( img_current, bound.tl(), bound.br(), cv::Scalar(0, 128, 0), 2, 8, 0 );
    }
  }
  //boundBox = boundRect;
  num_detect_ = boundBox.size();
  //box_past_ = boundBox;
}
/*
void MOTION::noise_judge(vector<Rect>& boundBox)
{
  if(box_past_.size() == boundBox.size())
  {
    for(int i = 0; i <>)
  }
}
*/
void MOTION::adjust()
{
  if(!auto_adjust) return;
  if(num_detect_ < num_detect - 2)
  {
    thresh -= 1;
    thresh = (thresh < 1)? 1:thresh;
  }
  else if(num_detect_ > num_detect + 2)
  {
    thresh += 1;
  }
  else
  {
  }
  
  if(num_detect_ - num_detect_past_ > num_detect_diff)
  {
    bg_filter_ksize += 2;
  }
  else if(num_detect_ - num_detect_past_ == num_detect_diff)
  {
  }
  else
  {
    bg_filter_ksize -= 2;
    bg_filter_ksize = (bg_filter_ksize < 1)? 3 : bg_filter_ksize;
  }
  num_detect_past_ = num_detect_;
}

#endif
