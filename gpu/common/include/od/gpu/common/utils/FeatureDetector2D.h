//
// Created by sarkar on 20.04.15.
//
#pragma once
#include "od/common/utils/FeatureDetectorInterface.h"
#include <opencv2/cudafeatures2d.hpp>
#include <GL/gl.h>
#include "SiftGPU.h"


namespace od
{

  namespace gpu 
  {

    class FeatureDetector2D : public FeatureDetectorIterface
    {

    public:

      FeatureDetector2D(FeatureType type);

      void computeKeypointsAndDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);

      void computeAndSave(const cv::Mat & image, const std::string & path);

      void findSiftGPUDescriptors(const std::string & image_name, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);

      void findSiftGPUDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);

    private:

      void findSiftGPUDescriptors_(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);

      cv::Ptr<SiftGPU> sift_gpu_;
      
    };

  }

}

