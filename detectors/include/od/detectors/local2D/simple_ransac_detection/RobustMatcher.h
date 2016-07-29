/*
 * RobustMatcher.h
 *
 *  Created on: Jun 4, 2014
 *      Author: eriba
 */
#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/ml.hpp>
#include "od/common/utils/FeatureDetector2D.h"
#include "od/common/utils/Utils.h"
#include "od/detectors/local2D/simple_ransac_detection/Model.h"
#include "od/detectors/local2D/simple_ransac_detection/Utils.h"


namespace od {
  
  namespace l2d {

    class
    RobustMatcher {
      
    public:
      RobustMatcher(Model const & model, bool use_gpu, bool b);

      virtual ~RobustMatcher();

      // Set the feature detector
      void setFeatureDetector(shared_ptr<cv::FeatureDetector> detect);

      // Set the descriptor extractor
      void setDescriptorExtractor(shared_ptr<cv::DescriptorExtractor> desc);

      // Set the matcher
      void setDescriptorMatcher(shared_ptr<cv::DescriptorMatcher> match);

      // Compute the keypoints of an image
      void computeKeyPoints(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints);

      // Compute the descriptors of an image given its keypoints
      void computeDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors);

      // Set ratio parameter for the ratio test
      void setRatio(float rat);

      // Clear matches for which NN ratio is > than threshold
      // return the number of removed points
      // (corresponding entries being cleared,
      // i.e. size will be 0)
      int ratioTest(std::vector<std::vector<cv::DMatch> > & matches);

      // Insert symmetrical matches in symMatches vector
      void symmetryTest(const std::vector<std::vector<cv::DMatch> > & matches1,
                        const std::vector<std::vector<cv::DMatch> > & matches2,
                        std::vector<cv::DMatch> & symMatches );

      // Match feature points using ratio and symmetry test
      void robustMatch(const cv::Mat & frame, std::vector<cv::DMatch> & good_matches,
                       std::vector<cv::KeyPoint> & keypoints_frame,
                       const cv::Mat & descriptors_model );

     // Match feature points using ratio test
     void fastRobustMatch(const cv::Mat & frame, std::vector<cv::DMatch> & good_matches,
                          std::vector<cv::KeyPoint> & keypoints_frame,
                          const cv::Mat & descriptors_model );

      void findFeatureAndMatch(const cv::Mat & frame, std::vector<cv::DMatch> & good_matches, std::vector<cv::KeyPoint> & keypoints_frame,
                               const cv::Mat & descriptors_model);

      void match(const cv::Mat & descriptors_frame, const cv::Mat & descriptors_model, std::vector<cv::DMatch> & good_matches);

      void matchNormalized(cv::Mat & descriptors_frame, cv::Mat & descriptors_model, std::vector<cv::DMatch> & good_matches);

      int mode_;
      bool use_gpu_;

    private:

      void instantiateMatcher(const Model & feature_type, bool use_gpu);
      void instantiateMatcher1(const Model & model, bool use_gpu);

      // pointer to the feature point detector object
      shared_ptr<od::FeatureDetector2D> featureDetector_;

      shared_ptr<cv::FeatureDetector> detector_;
      // pointer to the feature descriptor extractor object
      shared_ptr<cv::DescriptorExtractor> extractor_;
      // pointer to the matcher object

      //The important feature of this class starts here
      //DIFFERENT TYPES OF MATCHERS, add here in this list (based on the construction
      shared_ptr<cv::DescriptorMatcher> matcher_;
      cv::Ptr<cv::cuda::DescriptorMatcher> matcher_gpu_;
      // max ratio between 1st and 2nd NN
      float ratio_;  
    };

  }

}

