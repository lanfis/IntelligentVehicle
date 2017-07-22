#pragma once
#ifndef _DISPARITY_H_
#define _DISPARITY_H_

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"


class DISPARITY
{
    public:
      DISPARITY();
      ~DISPARITY();
      void run(Mat& img_left, Mat& img_right, Mat& img_disp);
      void init();

    private:
      int number_of_image_channels;
      int minDisparity;//Minimum possible disparity value. Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly. 
      int numDisparities;//Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter must be divisible by 16. 
      int blockSize;//Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range. 
      int P1;//The first parameter controlling the disparity smoothness. See below. 
      int P2;//The second parameter controlling the disparity smoothness. The larger the values are, the smoother the disparity is. P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels. P2 is the penalty on the disparity change by more than 1 between neighbor pixels. The algorithm requires P2 > P1 . See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown (like 8*number_of_image_channels*blockSize*blockSize and 32*number_of_image_channels*blockSize*blockSize , respectively). 
      int disp12MaxDiff;//Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check. 
      int preFilterCap;//Truncation value for the prefiltered image pixels. The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function. 
      int uniquenessRatio;//Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough. 
      int speckleWindowSize;//Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range. 
      int speckleRange;//Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough. 
      int mode;//Set it to StereoSGBM::MODE_HH to run the full-scale two-pass dynamic programming algorithm. It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures. By default, it is set to false .
    
    public:
      void setMinDisparity(int num){minDisparity = num;init();}
      void setNumDisparities(int num){numDisparities = num;init();}
      void setBlockSize(int num){blockSize = num;init();}
      void setP1(int num){P1 = num;init();}
      void setP2(int num){P2 = num;init();}
      void setDisp12MaxDiff(int num){disp12MaxDiff = num;init();}
      void setPreFilterCap(int num){preFilterCap = num;init();}
      void setUniquenessRatio(int num){uniquenessRatio = num;init();}
      void setSpeckleWindowSize(int num){speckleWindowSize = num;init();}
      void setSpeckleRange(int num){speckleRange = num;init();}
      void setMode(int num){mode = num;init();}
      
    private:
      Ptr<StereoSGBM> sgbm;
};

DISPARITY::DISPARITY()
{
    number_of_image_channels = 1;
    numDisparities = 64;//Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter must be divisible by 16. 
    minDisparity = -1*numDisparities/2;//Minimum possible disparity value. Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly. 
    blockSize = 3;//Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range. 
    P1 = 8*number_of_image_channels*blockSize*blockSize;//The first parameter controlling the disparity smoothness. See below. 
    P2 = 32*number_of_image_channels*blockSize*blockSize;//The second parameter controlling the disparity smoothness. The larger the values are, the smoother the disparity is. P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels. P2 is the penalty on the disparity change by more than 1 between neighbor pixels. The algorithm requires P2 > P1 . See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown (like 8*number_of_image_channels*blockSize*blockSize and 32*number_of_image_channels*blockSize*blockSize , respectively). 
    disp12MaxDiff = 1;//Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check. 
    preFilterCap = 10;//Truncation value for the prefiltered image pixels. The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function. 
    uniquenessRatio = 5;//Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough. 
    speckleWindowSize = 1000;//Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range. 
    speckleRange = 1;//Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough. 
    mode = StereoSGBM::MODE_SGBM;//Set it to StereoSGBM::MODE_HH to run the full-scale two-pass dynamic programming algorithm. It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures. By default, it is set to false .
    init();
}

DISPARITY::~DISPARITY()
{}

void DISPARITY::init()
{
    sgbm =  StereoSGBM::create(minDisparity,  numDisparities, blockSize, P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, mode);
}

void DISPARITY::run(Mat& img_left, Mat& img_right, Mat& img_disp)
{
    //if(!(img_left.channels() == 1 && img_right.channels() == 1)) return;
    if(img_left.empty() || img_right.empty())
    {
      return ;
    }
    sgbm -> compute(img_left, img_right, img_disp);
}

#endif
