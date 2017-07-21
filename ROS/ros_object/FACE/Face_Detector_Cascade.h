#pragma once
#ifndef _FACE_DETECTOR_CASCADE_H_
#define _FACE_DETECTOR_CASCADE_H_

#include <iostream>
#include <string>
#include <cstring>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


using namespace std;
using namespace cv;


const string fullbody_cascade_name = "/home/adel/opencv/data/haarcascades/haarcascade_fullbody.xml";
const string upperbody_cascade_name = "/home/adel/opencv/data/haarcascades/haarcascade_upperbody.xml";
const string face_cascade_name = "/home/adel/opencv/data/haarcascades/haarcascade_frontalface_alt.xml";
const string eyesplit_cascade_name = "/home/adel/opencv/data/haarcascades/haarcascade_lefteye_2splits.xml";
const string smile_cascade_name = "/home/adel/opencv/data/haarcascades/haarcascade_smile.xml";
const string eyeglass_cascade_name = "/home/adel/opencv/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";

class Face_Detector_Cascade
{
  private:
    CascadeClassifier fullbody_cascade;
    CascadeClassifier upperbody_cascade;
    CascadeClassifier face_cascade;
    CascadeClassifier eyesplit_cascade;
    CascadeClassifier smile_cascade;
    CascadeClassifier eyeglass_cascade;

  public:
    vector<Rect> fullbody;
    vector<Rect> upperbody;
    vector<Rect> face;
    vector<Rect> eyesplit;
    vector<Rect> smile;
    vector<Rect> eyeglass;

  public:
    Face_Detector_Cascade();
    ~Face_Detector_Cascade();
    void fullbody_detect(Mat& image);
    void upperbody_detect(Mat& image);
    void face_detect(Mat& image);
    void eyesplit_detect(Mat& image);
    void smile_detect(Mat& image);
    void eyeglass_detect(Mat& image);
};


Face_Detector_Cascade::Face_Detector_Cascade()
{
  if( !fullbody_cascade.load( fullbody_cascade_name ) ){ cout << "Error loading "<< fullbody_cascade_name << "\n";};
  if( !upperbody_cascade.load( upperbody_cascade_name ) ){ cout << "Error loading "<< upperbody_cascade_name << "\n";};
  if( !face_cascade.load( face_cascade_name ) ){ cout << "Error loading "<< face_cascade_name << "\n";};
  if( !eyesplit_cascade.load( eyesplit_cascade_name ) ){ cout << "Error loading "<< eyesplit_cascade_name << "\n";};
  if( !smile_cascade.load( smile_cascade_name ) ){ cout << "Error loading "<< smile_cascade_name << "\n";};
  if( !eyeglass_cascade.load( eyeglass_cascade_name ) ){ cout << "Error loading "<< eyeglass_cascade_name << "\n";};
  //return true;
}

Face_Detector_Cascade::~Face_Detector_Cascade()
{}

void Face_Detector_Cascade::fullbody_detect(Mat& image)
{
  fullbody_cascade.detectMultiScale( image, fullbody, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
}

void Face_Detector_Cascade::upperbody_detect(Mat& image)
{
  upperbody_cascade.detectMultiScale( image, upperbody, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
}

void Face_Detector_Cascade::face_detect(Mat& image)
{
  face_cascade.detectMultiScale( image, face, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
}

void Face_Detector_Cascade::eyesplit_detect(Mat& image)
{
  eyesplit_cascade.detectMultiScale( image, eyesplit, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
}

void Face_Detector_Cascade::smile_detect(Mat& image)
{
  smile_cascade.detectMultiScale( image, smile, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
}

void Face_Detector_Cascade::eyeglass_detect(Mat& image)
{
  eyeglass_cascade.detectMultiScale( image, eyeglass, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
}

#endif
