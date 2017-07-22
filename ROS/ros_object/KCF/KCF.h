#pragma once
#ifndef _KCF_H_
#define _KCF_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
/*
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"
*/
#include "kcftracker.hpp"

using namespace std;
using namespace cv;


class KCF
{
    public:
      KCF();
      ~KCF();
      bool run(Mat& image, Rect& roi);
      bool init(Mat& image, Rect& roi);
    
    public:
      float interp_factor; // linear interpolation factor for adaptation
      float sigma; // gaussian kernel bandwidth
      float lambda; // regularization
      int cell_size; // HOG cell size
      int cell_sizeQ; // cell size^2, to avoid repeated operations
      float padding; // extra area surrounding the target
      float output_sigma_factor; // bandwidth of gaussian target
      int template_size; // template size
      float scale_step; // scale step for multi-scale estimation
      float scale_weight;  // to downweight detection scores of other scales for added stability
      
    private:
      KCFTracker *kcft;
};

KCF::KCF()
{
    kcft = new KCFTracker;
    interp_factor = kcft -> interp_factor;
    sigma = kcft -> sigma;
    lambda = kcft -> lambda;
    cell_size = kcft -> cell_size;
    cell_sizeQ = kcft -> cell_sizeQ;
    padding = kcft -> padding;
    output_sigma_factor = kcft -> output_sigma_factor;
    template_size = kcft -> template_size;
    scale_step = kcft -> scale_step;
    scale_weight = kcft -> scale_weight;
}

KCF::~KCF()
{
    delete kcft;
}

bool KCF::init(Mat& image, Rect& roi)
{
    kcft -> interp_factor = interp_factor;
    kcft -> sigma = sigma;
    kcft -> lambda = lambda; // regularization
    kcft -> cell_size = cell_size; // HOG cell size
    kcft -> cell_sizeQ = cell_sizeQ; // cell size^2, to avoid repeated operations
    kcft -> padding = padding; // extra area surrounding the target
    kcft -> output_sigma_factor = output_sigma_factor; // bandwidth of gaussian target
    kcft -> template_size = template_size; // template size
    kcft -> scale_step = scale_step; // scale step for multi-scale estimation
    kcft -> scale_weight = scale_weight;  // to downweight detection scores of other scales for added stability
    if(!image.empty())
	{
      kcft -> init(roi, image);
	  return true;
	}
	else
	  return false;
}

bool KCF::run(Mat& image, Rect& roi)
{
    if(!image.empty())
	{
      roi = kcft -> update(image);
	  return true;
	}
    else
	  return false;
}

#endif
