#pragma once
#ifndef _MSER_H_
#define _MSER_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace std;
using namespace cv;

class MSER
{
  private:
    #define MSER_DELTA  5;
    #define MSER_MIN_WIDTH  3;
    #define MSER_MIN_HEIGHT  3;
    #define MSER_MAX_WIDTH  240;
    #define MSER_MAX_HEIGHT  240;
    #define MSER_MAX_VARIATION  1;
    #define MSER_MAX_DIVERSITY  0.01;
    #define MSER_MAX_EVOLUTION  1600;
    #define MSER_AREA_THRESHOLD  1.01;
    #define MSER_MIN_MARGIN  0.1;
    #define MSER_EDGE_BLUR_SIZE  1;    
    #define MSER_MIN_WINDOW_RATIO  0.05;
    #define MSER_MAX_WINDOW_RATIO  0.1;
      
    int min_width_ = MSER_MIN_WIDTH;
    int min_height_ = MSER_MIN_HEIGHT;
    int max_width_ = MSER_MAX_WIDTH;
    int max_height_ = MSER_MAX_HEIGHT;
    void para_adjust(Mat& image);
    void detect(Mat& image, vector< vector< Point> >& contours_, vector< Rect>& bboxes_);
    
  public:
    vector< vector< Point> > contours;
    vector< Rect> bboxes;
    vector< float> bboxes_angle;
    vector< Point2f> bboxes_center;
    vector< Size2f> bboxes_size;
    int delta = MSER_DELTA;
    int min_width = MSER_MIN_WIDTH;
    int min_height = MSER_MIN_HEIGHT;
    int max_width = MSER_MAX_WIDTH;
    int max_height = MSER_MAX_HEIGHT;
    double max_variation = MSER_MAX_VARIATION;
    double max_diversity = MSER_MAX_DIVERSITY;
    int max_evolution = MSER_MAX_EVOLUTION;
    double area_threshold = MSER_AREA_THRESHOLD;
    double min_margin = MSER_MIN_MARGIN;
    int edge_blur_size = MSER_EDGE_BLUR_SIZE;
    double min_window_ratio = MSER_MIN_WINDOW_RATIO;
    double max_window_ratio = MSER_MIN_WINDOW_RATIO;

  public:
    MSER();
    ~MSER();
    void init();
    void region_detect(Mat& image);//, vector< vector< Point> >& contours, vector< Rect>& bboxes, vector< Point2f>& center, vector< float>& angle, vector< Size2f>& size);
    void draw(Mat& image, Scalar color);
};


MSER::MSER()
{
}

MSER::~MSER()
{
}

void MSER::init()
{
  //para_adjust();
}

void MSER::para_adjust(Mat& image)
{
  int image_width = image.cols;
  int image_height = image.rows;
  min_width_ = (min_width < image_width * min_window_ratio)? min_width : image_width * min_window_ratio;
  min_height_ = (min_height < image_height * min_window_ratio)? min_height : image_height * min_window_ratio;
  max_width_ = (max_width < image_width * max_window_ratio)? max_width : image_width * max_window_ratio;
  max_height_ = (max_height < image_height * max_window_ratio)? max_height : image_height * max_window_ratio;
}

void MSER::region_detect(Mat& image)/*
                                , vector< vector< Point> >& contours
                                , vector< Rect>& bboxes 
                                , vector< Point2f>& center
                                , vector< float>& angle
                                , vector< Size2f>& size)
*/
{
  para_adjust(image);
  detect(image, contours, bboxes);
  
  bboxes_angle.clear();
  bboxes_center.clear();
  bboxes_size.clear();
  for (int i = 0; i < bboxes.size(); i++)
  {
    RotatedRect min_area = minAreaRect(contours[i]);
    bboxes_angle.push_back(min_area.angle);
    bboxes_center.push_back(min_area.center);
    bboxes_size.push_back(min_area.size);
  }
  return;
}

void MSER::detect(Mat& image, vector< vector< Point> >& contours, vector< Rect>& bboxes)
{
  int min_area = min_width_ * min_height_;
  int max_area = max_width_ * max_height_;
  if(image.channels() == 3)
  {
    Ptr< MSER> mser
   = MSER::create(delta
                , min_area
                , max_area
                , max_variation
                , max_diversity
                , max_evolution
                , area_threshold
                , min_margin
                , edge_blur_size
                );
    mser->detectRegions(image, contours, bboxes); 
  }
  else if(image.channels() == 1)
  {
    Ptr< MSER> mser
   = MSER::create(delta
                , min_area
                , max_area
                , max_variation
                );
    mser->detectRegions(image, contours, bboxes); 
  }
}

void MSER::draw(Mat& image, Scalar color)
{
  bboxes_angle.clear();
  bboxes_center.clear();
  bboxes_size.clear();
  for (int i = 0; i < bboxes.size(); i++)
  {
    RotatedRect min_area = minAreaRect(contours[i]);
    bboxes_angle.push_back(min_area.angle);
    bboxes_center.push_back(min_area.center);
    bboxes_size.push_back(min_area.size);
    
   
    circle(image, min_area.center, 1, color, 1, 4, 0);
      
    Point2f vtx[4];
    min_area.points(vtx);
    for(int j = 0; j < 4; j++)
      line(image, vtx[j], vtx[(j+1)%4], color, 1, LINE_AA);
    
//  rectangle(image, bboxes_[i], color, BOX_LENGTH);
  }
}
#endif
