#pragma once
#ifndef _FACE_DETECTOR_CASCADE_H_
#define _FACE_DETECTOR_CASCADE_H_

#include <iostream>
#include <string>
#include <cstring>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
using namespace boost::filesystem;

using namespace std;
using namespace cv;

//const string static_path = "/../ROS/ros_object/FACE/data";
const string static_path = "/data";

class Face_Detector_Cascade
{
  private:
    string data_directory = "/haarcascades";
    string fullbody_cascade_name = "/haarcascade_fullbody.xml";
    string upperbody_cascade_name = "/haarcascade_upperbody.xml";
    string face_cascade_name = "/haarcascade_frontalface_alt.xml";
    string eyesplit_cascade_name = "/haarcascade_lefteye_2splits.xml";
    string smile_cascade_name = "/haarcascade_smile.xml";
    string eyeglass_cascade_name = "/haarcascade_eye_tree_eyeglasses.xml";
	
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
  boost::filesystem::path path_curt = current_path();
  data_directory = path_curt.string() + static_path + data_directory;
  boost::filesystem::path dir(data_directory.c_str());
  if(!is_directory(dir))
    boost::filesystem::create_directories(dir);
  fullbody_cascade_name = data_directory + fullbody_cascade_name;
  upperbody_cascade_name = data_directory + upperbody_cascade_name;
  face_cascade_name = data_directory + face_cascade_name;
  eyesplit_cascade_name = data_directory + eyesplit_cascade_name;
  smile_cascade_name = data_directory + smile_cascade_name;
  eyeglass_cascade_name = data_directory + eyeglass_cascade_name;

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
