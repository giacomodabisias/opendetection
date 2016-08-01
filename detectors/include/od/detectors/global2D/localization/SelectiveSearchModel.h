#pragma once

#include "od/common/pipeline/Detector.h"
#include "od/common/pipeline/Scene.h"
#include "od/common/utils/Utils.h"
#include "od/common/utils/FeatureDetector2D.h"
#include "od/common/utils/Shared_pointers.h"

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <cstdlib>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <cstring>
#include <sstream>
#include <string>
#include <time.h>

namespace od
{
	namespace g2d
	{
		class SelectiveSearchModel: public Detector2D
		{
		public:

			SelectiveSearchModel(const std::string & trained_data_location = std::string(""));

			void setLabel(int l);
			void setBoundaries(int ix, int iy, int ax, int ay);
	
			void init();
			shared_ptr<Detections2D> detectOmni(shared_ptr<SceneImage> scene);
  			shared_ptr<Detections> detect(shared_ptr<SceneImage> scene);
  			shared_ptr<Detections> detectOmni(shared_ptr<Scene> scene); 

  			int min_x;
  			int max_x;
  			int min_y;
  			int max_y;
  			bool validity;
  			int size;

      		int label;

      		cv::Mat xx_hist, xy_hist, yy_hist, orientation_image_hist, differential_excitation_hist, color_hist;
      		std::vector<int> neighbors;

		};

		cv::Mat get_hess_hist_xx(const cv::Mat & regionMask, int histSize, float hist_range_min, float hist_range_max);
		
		cv::Mat get_hess_hist_xy(const cv::Mat & regionMask, int histSize, float hist_range_min, float hist_range_max);

		cv::Mat get_hess_hist_yy(const cv::Mat & regionMask, int histSize, float hist_range_min, float hist_range_max);

		cv::Mat get_orientation_hist(const cv::Mat & regionMask, int histSize, float hist_range_min, float hist_range_max);

		cv::Mat get_diff_exci_hist(const cv::Mat & regionMask, int histSize, float hist_range_min, float hist_range_max);

		void refineRegions(const std::vector<std::vector<int> > & sp, int total_masks, SelectiveSearchModel regions[], int min_height, int min_width);

		void createModel(const std::vector<std::vector<int> > & sp, int total_masks, const cv::Mat & grayMask, SelectiveSearchModel regions[], 
			             int histSize, float hist_range_min, float hist_range_max);

		bool checkNeighbors(const SelectiveSearchModel & a, const SelectiveSearchModel & b);

		std::vector<std::vector<int> > findNeighbors(SelectiveSearchModel regions[], int total_masks, cv::Mat & regionMask, std::vector<std::vector<int> > & sp);
		float calcSimilarities(const SelectiveSearchModel & a, const SelectiveSearchModel & b, float spSize);

		void mergeRegions(int value, SelectiveSearchModel regions[], std::vector<std::vector<int> > & sp_neighbors, cv::Mat & grayMask, 
			              const std::vector<std::vector<int> > & sp, int minRegionSize, int histSize, float hist_range_min, float hist_range_max);

		bool checkRounds(int totals, SelectiveSearchModel regions[], int numRounds);
		std::vector<std::vector<int> > extractROIs(int total_masks, SelectiveSearchModel regions[], int numRounds, float spSize, 
			                                       const std::vector<std::vector<int> > & sp, const cv::Mat & img, const cv::Mat & gray_mask, 
			                                       int minRegionSize, int histSize, float hist_range_min, float hist_range_max);
	}
}


