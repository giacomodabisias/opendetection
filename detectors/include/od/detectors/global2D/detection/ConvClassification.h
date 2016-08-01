#pragma once

#include "od/common/pipeline/Detector.h"
#include "od/common/pipeline/Scene.h"
#include "od/common/utils/Utils.h"
#include "od/common/utils/FeatureDetector2D.h"


#include <opencv2/opencv.hpp>

#include <cstring>
#include <cstdlib>
#include <vector>

#include <string>
#include <iostream>
#include <stdio.h>

#include <caffe/caffe.hpp>
#include <caffe/util/io.hpp>
#include <caffe/blob.hpp>

#include "od/common/utils/Shared_pointers.h"

namespace od
{
	namespace g2d
	{
		class ConvClassification : public Detector2D
		{
		public:	

			ConvClassification(const std::string & trained_data_location = std::string(""));

			void setWeightModelFileLocation(const std::string & location);
			void setNetworkModelFileLocation(const std::string & location);
			void setImageFileLocation(const std::string & location);

			std::string getWeightModelFileLocation();
			std::string getNetworkModelFileLocation();
			std::string getImageFileLocation();

			void setTestBlob(int num_channels, int img_height, int img_width);				
			void classify();

			void init();
			shared_ptr<Detections2D> detectOmni(shared_ptr<SceneImage> scene);
  			shared_ptr<Detections> detect(shared_ptr<SceneImage> scene);
  			shared_ptr<Detections> detectOmni(shared_ptr<Scene> scene);
    			
		private:
			std::string weightModelFileLoaction;
			std::string networkFileLocation;
			std::string imageFileLocation; 
			caffe::Datum strucBlob;
			caffe::BlobProto protoBlob;
			std::vector<caffe::Blob<float>*> inputBlob;

		};	
	}
}


