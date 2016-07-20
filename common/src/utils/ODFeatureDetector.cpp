#include "od/common/utils/ODFeatureDetector.h"

namespace od
{

  ODFeatureDetector::ODFeatureDetector(FeatureType type)
  {

    mode_ = type;

    switch(mode_)
    {
      case(SIFT) : 
      case(ORB) : 
      case(SURF) : 
        feature_detector_ = make_shared<ODFeatureDetector2D>(mode_);
        gpu_ = false;
        break;
      case(SIFT_GPU):
      case(ORB_GPU):
#if WIHT_GPU
        feature_detector_ = make_shared<od::gpu::ODFeatureDetector2D>(mode_);
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

  void ODFeatureDetector::computeKeypointsAndDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
  {
      feature_detector_->computeKeypointsAndDescriptors(image, descriptors,keypoints);
  }

  void ODFeatureDetector::computeAndSave(const cv::Mat & image, const std::string & path)
  {
      feature_detector_->computeAndSave(image, path);
  }

}