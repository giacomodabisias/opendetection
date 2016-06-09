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
// Created by sarkar on 05.06.15.
//

#include "od/detectors/local2D/ODImageLocalMatching.h"

namespace od
{

  namespace l2d
  {

	   ODImageLocalMatchingTrainer::ODImageLocalMatchingTrainer(const std::string & training_input_location_, 
	   	                                                        const std::string & training_data_location_) : 
	   ODTrainer(training_input_location_, training_data_location_)
	   {
	     	TRAINED_LOCATION_DENTIFIER_ = "FEATCORR";
	     	TRAINED_DATA_ID_ = "corr.xml";
	   }

		ODImageLocalMatchingDetector::ODImageLocalMatchingDetector(const std::string & training_data_location_) : 
		ODDetector2DComplete(training_data_location_)
		{
			TRAINED_LOCATION_DENTIFIER_ = "FEATCORR";
		 	TRAINED_DATA_ID_ = ".xml";
		}


	   shared_ptr<ODImageLocalMatchingTrainer> ODImageLocalMatching::getTrainer() const
	   {
	     return trainer_;
	   }

	   void ODImageLocalMatching::setTrainer(shared_ptr<ODImageLocalMatchingTrainer> trainer_)
	   {
	     ODImageLocalMatching::trainer_ = trainer_;
	   }

	   shared_ptr<ODImageLocalMatchingDetector> ODImageLocalMatching::getDetector() const
	   {
	     return detector_;
	   }

	   void ODImageLocalMatching:: setDetector(shared_ptr<ODImageLocalMatchingDetector> detector_)
	   {
	     ODImageLocalMatching::detector_ = detector_;
	   }

	   ODImageLocalMatching::ODImageLocalMatching()
	   {
	     TRAINED_DATA_EXT_ = "corr.xml";
	   }

	   int ODImageLocalMatching::train()
	   {
	     return trainer_->train();
	   }

	   int ODImageLocalMatching::detect(shared_ptr<ODScene> scene, std::vector<shared_ptr<ODDetection> > detections)
	   {
	     //detector_->detect(scene, detections);
	     return 1;
	   }
	}
}