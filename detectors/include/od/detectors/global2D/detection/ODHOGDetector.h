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
// Created by sarkar on 15.07.15.
//
#pragma once
#include "od/common/pipeline/ODDetector.h"
#include "od/common/pipeline/ODScene.h"
#include "od/common/utils/ODUtils.h"
#include "od/common/utils/ODFeatureDetector2D.h"
#include "od/common/utils/ODShared_pointers.h"
#include "od/common/bindings/svmlight.h"

#include <iostream>
#include <opencv2/objdetect.hpp>

namespace od
{

  namespace g2d
  {
    /** \brief A linear classifier for HOG features.
     *
     * This class takes an image as an input, finds its HOG features using the parameters containing in the trained data, runs the linear classifier (present in the trained data) on the computed HOG features
     * and finally produces the classification output. Covers both multiscale detection (detectOmni()) and detection on a fixed scene (detect() - the scene is resized to the HOG window size).
     *
     * Use ODHOGTrainer for the training of new objects. By default this class provides two people detector: OD_DEFAULT_PEOPLE, the people detector from OpenCV and OD_DAIMLER_PEOPLE, the second detector available from OpenCV.
     * One can set any linear classifier, the linear weight vector through setSVMDetector() function but it is highly not recommended as the HOG parameters and the weight vector can be out of sync.
     * One must set appropriate HOG parameters after using setSVMDetector() function with which the weight vector was trained.
     *
     * \author Kripasindhu Sarkar
     *
     */


    class ODHOGDetector : public ODDetector2D
    {
    public:

      ODHOGDetector(const std::string & trained_data_location_ = "", const cv::Size & win_size = cv::Size(64,128),
                    const cv::Size & block_size = cv::Size(16,16), const cv::Size & block_stride = cv::Size(8,8),
                    const cv::Size & cell_size = cv::Size(8,8), float hit_threshold = 0.0);

      OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(SVMType, (OD_CUSTOM)(OD_DEFAULT_PEOPLE)(OD_DAIMLER_PEOPLE)(OD_FILE))

      void init();
      void load(const std::string & file_name);

      void setSVMFromFile(const std::string & file_name);

      void setSVMDetector(std::vector<float> svm_detector);

      shared_ptr<ODDetections2D> detectOmni(shared_ptr<ODSceneImage> scene);
      shared_ptr<ODDetections> detect(shared_ptr<ODSceneImage> scene);

      int detect(shared_ptr<ODScene> scene, std::vector<shared_ptr<ODDetection> > & detections);

      void setTrainedDataLocation(const std::string & trained_data_location);

      const SVMType & getSvmtype() const;
      void setSvmtype(const SVMType & svm_type_);

      const cv::Size & getWinSize() const;
      void setWinSize(const cv::Size & win_size);

      const cv::Size & getBlockSize() const;
      void setBlockSize(const cv::Size & block_size);

      const cv::Size & getBlockStride() const;
      void setBlockStride(const cv::Size & block_stride);

      const cv::Size & getCellSize() const;
      void setCellSize(const cv::Size & cell_size);

      float getHitThreshold() const;
      void setHitThreshold(float hit_threshold);

      void printParameters();

    protected:
      //properteis
      cv::Size win_size_;
      cv::Size block_size_;
      cv::Size block_stride_;
      cv::Size cell_size_;

      float hit_threshold_;

      cv::HOGDescriptor hog_;
      SVMType svm_type_;

    };

  /** \examples objectdetector/od_image_hog.cpp
  *   \examples objectdetector/od_image_hog_files.cpp
  *   \examples apps/global2D/od_multihog_app.cpp
  *   This is an example of how to use the ODHOGDetector class.
  *
  *   More details about this example.
  */
  }
}

