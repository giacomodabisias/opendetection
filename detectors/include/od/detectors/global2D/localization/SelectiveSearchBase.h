#pragma once

#include "od/common/pipeline/Detector.h"
#include "od/common/pipeline/Scene.h"
#include "od/common/utils/Utils.h"
#include "od/common/utils/FeatureDetector2D.h"
#include "od/common/utils/Shared_pointers.h"
#include "od/detectors/global2D/localization/Image.h"
#include "od/detectors/global2D/localization/Misc.h"
#include "od/detectors/global2D/localization/Pnmfile.h"
#include "od/detectors/global2D/localization/Segment-image.h"

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
		class SelectiveSearchBase: public Detector2D
		{
		public:

			SelectiveSearchBase(const std::string & trained_data_location = std::string(""));

			void acquireImages(const std::string & image_location, int img_width, int img_height);
			cv::Mat preProcessImg(const cv::Mat & image);
			std::vector<std::vector<int> > getSuperPixels(const cv::Mat & im, int & total_masks, float sigma, float k, float min_size, 
				                                      const std::string & image_location);		 

			void init();
			shared_ptr<Detections2D> detectOmni(shared_ptr<SceneImage> scene);
  			shared_ptr<Detections> detect(shared_ptr<SceneImage> scene);

		private:

			cv::Mat img, cluster, outputImg, sp_preProcessed, gray_mask;
			int inputImageHeight;
			int inputImageWidth;
			int total_masks;
			std::vector<std::vector<int> > sp; 
		};
	}
}


