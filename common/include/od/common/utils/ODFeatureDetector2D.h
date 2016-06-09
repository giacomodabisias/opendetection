//
// Created by sarkar on 20.04.15.
//
#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/cudafeatures2d.hpp>

#include "SiftGPU.h"


namespace od
{

  enum FeatureType
  {
    SIFT, SURF, ORB, SIFT_GPU, ORB_GPU,
  };

  class ODFeatureDetector2D
  {

  public:
    ODFeatureDetector2D(const std::string & feature_type, bool use_gpu);

    ODFeatureDetector2D(FeatureType type);

    void computeKeypointsAndDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);

    void findSiftGPUDescriptors(const std::string & image_name, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);

    void findSiftGPUDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);

    void computeAndSave(const cv::Mat & image, const std::string & path);

  private:

    void findSiftGPUDescriptors_(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);

    cv::Ptr<cv::FeatureDetector> feature_detector_;
    cv::Ptr<SiftGPU> sift_gpu_;
    int mode_;
  };

}

