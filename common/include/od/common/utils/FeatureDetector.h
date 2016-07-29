#pragma once
#include "od/common/utils/Shared_pointers.h"
#include "od/common/utils/FeatureDetectorInterface.h"
#include "od/common/utils/FeatureDetector2D.h"



namespace od
{

	class FeatureDetector
	{

	public:

	  FeatureDetector(FeatureType type);
	  FeatureDetector(const std::string & type);

	  void computeKeypointsAndDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints);

	  void computeAndSave(const cv::Mat & image, const std::string & path);

	private:

	  FeatureType mode_;
	  bool gpu_;
	  shared_ptr<FeatureDetectorIterface> feature_detector_;

	};

}