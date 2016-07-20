//
// Created by sarkar on 20.04.15.
//

#include "od/common/utils/ODFeatureDetector2D.h"

namespace od
{

  ODFeatureDetector2D::ODFeatureDetector2D(FeatureType type)
  {

    mode_ = type;

    switch(type)
    {
      case(SIFT) : 
        feature_detector_ = cv::xfeatures2d::SIFT::create();
        break;
      
      case(ORB) : 
        feature_detector_ = cv::ORB::create();
        break;
      
      case(SURF) : 
        feature_detector_ = cv::xfeatures2d::SURF::create();
        break;

      case(ORB_GPU) :
      case(SIFT_GPU) :
      default :
        std::cout << "FATAL ERROR, type is not defined" << std::endl;
        exit(-1);
        break;
      
    }
  }

  void ODFeatureDetector2D::computeKeypointsAndDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
  {

      feature_detector_->detect(image, keypoints);
      feature_detector_->compute(image, keypoints, descriptors);
  }


  void ODFeatureDetector2D::computeAndSave(const cv::Mat & image, const std::string & path)
  {
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
   
    //TODO implementation
    
  }
}
