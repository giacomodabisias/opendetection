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
// Created by sarkar on 05.06.15.
//

#include "od/detectors/local2D/ImageLocalMatching.h"

namespace od
{

  namespace l2d
  {

		ImageLocalMatchingTrainer::ImageLocalMatchingTrainer(const std::string & training_input_location, 
			                                                        const std::string & training_data_location) : 
		Trainer(training_input_location, training_data_location)
		{
		 	trained_location_identifier_ = std::string("FEATCORR");
		 	trained_data_id_ = std::string("corr.xml");
		}

		ImageLocalMatchingDetector::ImageLocalMatchingDetector(const std::string & training_data_location) : 
																	Detector2DComplete(training_data_location)
		{
			trained_location_identifier_ = std::string("FEATCORR");
		 	trained_data_id_ = std::string(".xml");
		}

		shared_ptr<ImageLocalMatchingTrainer> ImageLocalMatching::getTrainer() const
		{
		 return trainer_;
		}

		void ImageLocalMatching::setTrainer(shared_ptr<ImageLocalMatchingTrainer> trainer)
		{
		 trainer_ = trainer;
		}

		shared_ptr<ImageLocalMatchingDetector> ImageLocalMatching::getDetector() const
		{
		 return detector_;
		}

		void ImageLocalMatching:: setDetector(shared_ptr<ImageLocalMatchingDetector> detector)
		{
		 detector_ = detector;
		}

		ImageLocalMatching::ImageLocalMatching()
		{
		 trained_data_ext_ = std::string("corr.xml");
		}

		int ImageLocalMatching::train()
		{
		 return trainer_->train();
		}

		int ImageLocalMatching::detect(shared_ptr<Scene> scene, const std::vector<shared_ptr<Detection> > & detections)
		{
		 //detector_->detect(scene, detections);
		 return 1;
		}
	}
}