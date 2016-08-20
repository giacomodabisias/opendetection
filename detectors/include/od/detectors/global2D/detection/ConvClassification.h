#pragma once

#include "od/common/pipeline/Detector.h"
#include "od/common/pipeline/Scene.h"
#include "od/common/utils/Shared_pointers.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>

#include <caffe/caffe.hpp>
#include <caffe/util/io.hpp>
#include <caffe/blob.hpp>


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
			void setOutputFileLocation(const std::string & location);

			std::string getWeightModelFileLocation();
			std::string getNetworkModelFileLocation();
			std::string getImageFileLocation();

			std::vector<float> classifyMultiLabel();
			void setTestBlob(int num_channels, int img_height, int img_width);				
			void classify();

			//Multiclass classification . Note: Code Partially taken from caffe library
			std::vector<float> runMultiClassClassifier();
			int runMultiClassClassifierPythonMode();
			std::vector<float> Predict(const cv::Mat & img);
			void WrapInputLayer(std::vector<cv::Mat> & input_channels); 

			//Segnet classification
			void setSegnetLocation(const std::string & location);
			void setImageGroundTruthFileLocation(const std::string & location);
			void setColorLocation(const std::string & location);
			int runSegnetBasedClassifierPythonMode();

			void init();
			shared_ptr<Detections2D> detectOmni(shared_ptr<SceneImage> scene);
  			shared_ptr<Detections> detect(shared_ptr<SceneImage> scene);
  			shared_ptr<Detections> detectOmni(shared_ptr<Scene> scene);
    			
		private:
			std::string imageFileLocation, outputFileLocation, networkFileLocation, weightModelFileLoaction, segnetLocation, colorLocation, imageGroundTruthFileLocation; 
			caffe::Datum strucBlob;
			caffe::BlobProto protoBlob;
			std::vector<caffe::Blob<float>*> inputBlob;

			cv::Size input_geometry_;
			int num_channels_;


		};	
	}
}


