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
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by sarkar on 03.06.15.
//

#include "od/common/pipeline/ObjectDetector.h"

namespace od {

	DetectorCommon::DetectorCommon(const std::string & trained_data_location) : trained_data_location_(trained_data_location)
	{
	  std::string classname = typeid(this).name();
	  trained_data_id_ = classname;
	  std::transform(classname.begin(), classname.end(), classname.begin(), ::toupper);
	  trained_location_identifier_  = classname;
	}

	std::string DetectorCommon::getTrainingInputLocation() const
	{
	  return training_input_location_;
	}

	void DetectorCommon::setTrainingInputLocation(const std::string & training_input_location)
	{
	  training_input_location_ = training_input_location;
	}

	std::string DetectorCommon::getTrainedDataLocation() const
	{
	  return trained_data_location_;
	}

	void DetectorCommon::setTrainedDataLocation(const std::string & trained_data_location)
	{
	  trained_data_location_ = trained_data_location;
	}

	std::string DetectorCommon::getSpecificTrainingDataLocation()
	{
	  return trained_data_location_ + "/" + "TD_" + trained_location_identifier_;
	}

	std::string DetectorCommon::getSpecificTrainingData()
	{
	  return getSpecificTrainingDataLocation() + "/" + trained_data_id_;
	}

	const std::string & DetectorCommon::getTrainedDataID() const
	{
	  return trained_data_id_;
	}

	void DetectorCommon::setTrainedDataID(const std::string & trainedDataID)
	{
	  trained_data_id_ = trainedDataID;
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
	  return training_data_location_ + "/" + "TD_" + trained_data_identifier_;
	}

}