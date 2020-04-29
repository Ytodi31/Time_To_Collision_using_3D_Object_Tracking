
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
  std::vector<cv::DMatch>tempMatches;
  for(auto it = kptMatches.begin(); it!=kptMatches.end(); it++)
  {
    if(boundingBox.roi.contains(kptsCurr[it->trainIdx].pt))
      tempMatches.push_back(*it);
  }
  float bb_centreX = boundingBox.roi.x + boundingBox.roi.width/2;
  float bb_centreY = boundingBox.roi.y + boundingBox.roi.height/2;
  std::vector<double> distances;

  double distance_thresh = sqrt(pow(0.9*boundingBox.roi.width/2,2) + pow(0.9*boundingBox.roi.height/2,2));
  
  for(auto it = tempMatches.begin(); it!=tempMatches.end(); it++)
  {
    double distance = sqrt(pow(bb_centreX - kptsCurr[it->trainIdx].pt.x,2) + pow(bb_centreY - kptsCurr[it->trainIdx].pt.y,2));
	if (distance <= distance_thresh){
    	distances.push_back(distance);
        boundingBox.kptMatches.push_back(*it);
    } 
  }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
  	vector<double> distRatios;
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end(); it1++)
    {
     	cv::KeyPoint curr1 = kptsCurr[it1->trainIdx];
      	cv::KeyPoint prev1 = kptsPrev[it1->queryIdx];
      
      for(auto it2 = kptMatches.begin() +1; it2 != kptMatches.end(); ++it2)
      {
        double minDist = 100;
        cv::KeyPoint curr2 = kptsCurr[it2->trainIdx];
      	cv::KeyPoint prev2 = kptsPrev[it2->queryIdx];
        
         double distCurr = cv::norm(curr1.pt - curr2.pt);
         double distPrev = cv::norm(prev1.pt - prev2.pt);
        
         if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { 

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
      }
    }
   // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }
	double medianDistRatio = 0;
     sort(distRatios.begin(), distRatios.end());
  	if (distRatios.size()%2 ==0)
      medianDistRatio = distRatios[distRatios.size()/2];
  	else
      medianDistRatio = distRatios[(distRatios.size()+1)/2];
    
    double dT = 1 / frameRate;
    TTC = -dT / (1 -  medianDistRatio);   
    cout << "Time to collision from Camera : " << TTC << std ::endl;  
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{ 
  	std::vector<double> current;
  	std::vector<double> previous;
   for(auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); it++)
    {
      previous.push_back(it->x);
    }
   for(auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); it++)
    {
      current.push_back(it->x);
    }
    sort(previous.begin(), previous.end());
  	sort(current.begin(), current.end());
    int curr_size = static_cast<int>(current.size()*0.10);
    int prev_size = static_cast<int>(previous.size()*0.10);
    double minXCurr = accumulate(current.begin()+curr_size, current.begin()+curr_size+5, 0.0)/5;
    double minXPrev = accumulate(previous.begin()+prev_size, previous.begin()+prev_size+5, 0.0)/5;
//     double minXCurr = current.at(static_cast<int>(current.size()*0.10));
//     double minXPrev = previous.at(static_cast<int>(previous.size()*0.10));
  	try{
    TTC = minXCurr/(frameRate*(minXPrev - minXCurr));
  	}
   	catch(...){
     TTC = 0;
   	}
    cout << "Time to collision from LiDAR : " << TTC << std ::endl;  
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
  std::multimap<int, int> mm;
  auto train_bb = currFrame;
  auto query_bb = prevFrame;
  int prev_boxes = query_bb.boundingBoxes.size();
  int curr_boxes = train_bb.boundingBoxes.size();
     for(auto it = matches.begin(); it < matches.end();it++)
       { 
		for(auto bb1 = train_bb.boundingBoxes.begin(); bb1 < train_bb.boundingBoxes.end(); ++bb1)
        {
          if((*bb1).roi.contains(train_bb.keypoints[(*it).trainIdx].pt))
          {
            for(auto bb2 = query_bb.boundingBoxes.begin(); bb2 < query_bb.boundingBoxes.end(); ++bb2)
            {
            	if((*bb2).roi.contains(query_bb.keypoints[(*it).queryIdx].pt))
                { 
                  mm.insert(std::pair<int,int>(bb2->boxID, bb1->boxID));
                } // if of bb2
            } // for of bb2
          } //if of bb1
          } // for of bb1
      }//matches

  for(auto prev_id = 0; prev_id < prev_boxes; prev_id++)
  {
//     cout << "Previous frame box ID:" << prev_id << " with count " << mm.count(prev_id) << std::endl;
    auto best_boxes = mm.equal_range(prev_id);
    std::vector<int> curr_box;
    for(auto it = best_boxes.first; it!= best_boxes.second; ++it)
  	{
		if(it->first == prev_id)
		curr_box.push_back(it->second);
   	}
    int max_countCurr = 0;
    int best_currBox = 0;
    for(auto curr_id = 0; curr_id < curr_boxes; curr_id++)
    {
    	int temp_count = std::count(curr_box.begin(), curr_box.end(), curr_id); 
//         std::cout << "Curr box ID: " << curr_id << "with count " << temp_count << std::endl;
      	if(temp_count > max_countCurr)
    	{
       		max_countCurr = temp_count;
       		best_currBox  = curr_id;
    	 }
    }
    bbBestMatches.insert(std::pair<int,int>(prev_id, best_currBox)); 
  } 
}