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
*///
// Created by sarkar on 17.07.15.
//
#pragma once
#include "od/common/pipeline/ODDetector.h"
#include "od/common/pipeline/ODScene.h"
#include "od/common/utils/ODUtils.h"
#include "od/common/utils/ODFeatureDetector2D.h"

#include <opencv2/opencv.hpp>

namespace od
{
  namespace g2d
  {
    /** \brief A class for detection using Cascade classifiers.
     * Given a scene and a cascade classifier, this class performs a classification and returns detections. The training is not supported in OD currently but is compatible to the cascade training of OpenCV.
     * Train your cascade classifiers using OpenCV's *opencv_traincascade* utility (http://docs.opencv.org/master/dc/d88/tutorial_traincascade.html#gsc.tab=0). It is a great tool for traning your cascade.
     * Paste the generated xml in trained_data_location_/TD_CASCADE/\*.cascade.xml to use your trained cascade.
     *
     * \author Kripasindhu Sarkar
     *
     */

    class ODCascadeDetector : public ODDetector2D
    {
    public:

      ODCascadeDetector(const std::string & trained_data_location = "", double scale_factor = 1.1, int min_neighbors = 3, 
                        int flags = 0, const cv::Size & min_size = cv::Size(), const cv::Size & max_size = cv::Size());

      void init();

      shared_ptr<ODDetections2D> detectOmni(shared_ptr<ODSceneImage> scene);
      shared_ptr<ODDetections> detect(shared_ptr<ODSceneImage> scene);

    private:
      shared_ptr<cv::CascadeClassifier> haar_cascade_;

      double scale_factor_;
      int min_neighbors_;
      cv::Size min_size_;
      cv::Size max_size_;

    };
    /** \examples objectdetector/od_image_cascade.cpp
     *  \examples objectdetector/od_image_cascade_files.cpp
     */
  }

}
