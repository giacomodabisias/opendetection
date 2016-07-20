#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <map>

namespace od
{

	enum FeatureType
	{
	  SIFT, SURF, ORB, SIFT_GPU, ORB_GPU
	};

	extern std::map<std::string, FeatureType> od_enum_map;

	FeatureType string2FeatureType(const std::string & name);

	class ODFeatureDetectorIterface
	{

	public:

	  virtual void computeKeypointsAndDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints) = 0;

	  virtual void computeAndSave(const cv::Mat & image, const std::string & path) = 0;

	protected:

	  FeatureType mode_;
	  cv::Ptr<cv::FeatureDetector> feature_detector_;

	};

}