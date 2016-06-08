/*
Copyright (c) 2015, Kripasindhu Sarkar
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by sarkar on 03.06.15.
//

#include "od/common/pipeline/ODObjectDetector.h"

namespace od {

	ODDetectorCommon::ODDetectorCommon(const std::string & trained_data_location) : trained_data_location_(trained_data_location)
	{
	  std::string classname = typeid(this).name();
	  TRAINED_DATA_ID_ = classname;
	  std::transform(classname.begin(), classname.end(), classname.begin(), ::toupper);
	  TRAINED_LOCATION_DENTIFIER_  = classname;
	}

	std::string ODDetectorCommon::getTrainingInputLocation() const
	{
	  return training_input_location_;
	}

	void ODDetectorCommon::setTrainingInputLocation(const std::string & training_input_location)
	{
	  training_input_location_ = training_input_location;
	}

	std::string ODDetectorCommon::getTrainedDataLocation() const
	{
	  return trained_data_location_;
	}

	void ODDetectorCommon::setTrainedDataLocation(const std::string & trained_data_location)
	{
	  trained_data_location_ = trained_data_location;
	}

	std::string ODDetectorCommon::getSpecificTrainingDataLocation()
	{
	  return trained_data_location_ + "/" + "TD_" + TRAINED_LOCATION_DENTIFIER_;
	}

	std::string ODDetectorCommon::getSpecificTrainingData()
	{
	  return getSpecificTrainingDataLocation() + "/" + TRAINED_DATA_ID_;
	}

	const std::string & ODDetectorCommon::getTrainedDataID() const
	{
	  return TRAINED_DATA_ID_;
	}

	void ODDetectorCommon::setTrainedDataID(const std::string & trainedDataID)
	{
	  ODDetectorCommon::TRAINED_DATA_ID_ = trainedDataID;
	}


	const DetectionMethod & ObjectDetector::getMethod() const
	{
	  return method_;
	}

	void ObjectDetector::setDetectionMethod(const DetectionMethod & detection_method)
	{
	  method_ = detection_method;
	}

	bool ObjectDetector::getAlwaysTrain() const
	{
	  return always_train_;
	}

	void ObjectDetector::setAlwaysTrain(bool always_train)
	{
	  always_train_ = always_train;
	}

	std::string ObjectDetector::getTrainingInputLocation() const
	{
	  return training_input_location_;
	}

	void ObjectDetector::setTrainingInputLocation(const std::string & training_input_location)
	{
	  training_input_location_ = training_input_location;
	}

	std::string ObjectDetector::getTrainingDataLocation() const
	{
	  return training_data_location_;
	}

	void ObjectDetector::setTrainingDataLocation(const std::string & training_data_location)
	{
	  training_data_location_ = training_data_location;
	}

	std::string ObjectDetector::getSpecificTrainingDataLocation()
	{
	  return training_data_location_ + "/" + "TD_" + TRAINED_DATA_IDENTIFIER_;
	}

}