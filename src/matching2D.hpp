
#ifndef matching2D_hpp
#define matching2D_hpp

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"

double detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);
double detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);
double detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis=false);
double FastPointDetector(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis);
double BRISKPointDetector(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis);
double ORBPointDetector(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis);
double AKAZEPointDetector(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis);
double SIFTPointDetector(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis);


double descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType);
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType);
         


#endif /* matching2D_hpp */
