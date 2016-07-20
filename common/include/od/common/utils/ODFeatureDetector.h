#pragma once
#include "od/common/utils/ODShared_pointers.h"
#include "od/common/utils/ODFeatureDetectorInterface.h"
#include "od/common/utils/ODFeatureDetector2D.h"



namespace od
{

	class ODFeatureDetector
	{

	public:

	  ODFeatureDetector(FeatureType type);
	  ODFeatureDetector(const std::string & type);

	  void computeKeypointsAndDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);

	  void computeAndSave(const cv::Mat & image, const std::string & path);

	private:

	  FeatureType mode_;
	  bool gpu_;
	  shared_ptr<ODFeatureDetectorIterface> feature_detector_;

	};

}