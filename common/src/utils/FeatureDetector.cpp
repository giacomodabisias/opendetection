#include "od/common/utils/FeatureDetector.h"

namespace od
{

  FeatureDetector::FeatureDetector(FeatureType type)
  {

    mode_ = type;

    switch(mode_)
    {
      case(SIFT) : 
      case(ORB) : 
      case(SURF) : 
        feature_detector_ = make_shared<FeatureDetector2D>(mode_);
        gpu_ = false;
        break;
      case(SIFT_GPU):
      case(ORB_GPU):
#if WIHT_GPU
        feature_detector_ = make_shared<od::gpu::FeatureDetector2D>(mode_);
        gpu_ = true;
#else
        std::cout << "FATAL ERROR, gpu type is not compile. Recompile with WITH_GPU to enable GPU support." << std::endl;
        exit(-1);
#endif
      default :
          std::cout << "FATAL ERROR, type is not defined" << std::endl;
          exit(-1);
        break;
      
    }
  }

  FeatureDetector::FeatureDetector(const std::string & type)
  {
  	FeatureDetector(string2FeatureType(type));
  }

  void FeatureDetector::computeKeypointsAndDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
  {
      feature_detector_->computeKeypointsAndDescriptors(image, descriptors,keypoints);
  }

  void FeatureDetector::computeAndSave(const cv::Mat & image, const std::string & path)
  {
      feature_detector_->computeAndSave(image, path);
  }

}