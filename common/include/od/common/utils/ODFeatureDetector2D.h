//
// Created by sarkar on 20.04.15.
//
#pragma once
#include "od/common/utils/ODFeatureDetectorInterface.h"
#include <opencv2/xfeatures2d.hpp>


namespace od
{

  class ODFeatureDetector2D: public ODFeatureDetectorIterface
  {

  public:

    ODFeatureDetector2D(FeatureType type);

    void computeKeypointsAndDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);

    void computeAndSave(const cv::Mat & image, const std::string & path);

  };

}

