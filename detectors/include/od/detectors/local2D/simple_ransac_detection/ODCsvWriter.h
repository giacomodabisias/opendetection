#pragma once
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include "od/detectors/local2D/simple_ransac_detection/ODUtils.h"


namespace od {
  
  namespace l2d {

		class CsvWriter {
		public:
		  CsvWriter(const std::string & path, const std::string & separator = std::string(" "));
		  ~CsvWriter();
		  void writeXYZ(const std::vector<cv::Point3f> & list_points3d);
		  void writeUVXYZ(const std::vector<cv::Point3f> & list_points3d, const std::vector<cv::Point2f> & list_points2d, const cv::Mat & descriptors);

		private:

		  std::ofstream file_;
		  std::string separator_;
		  bool is_first_term_;

		};

	}
	
}

