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
// Created by sarkar on 13.08.15.
//
#pragma once
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "od/common/pipeline/ODTrainer.h"
#include "od/common/bindings/ODSvmlight.h"

namespace od
{
  namespace g2d
  {
    /** \brief Class for training HOG based detector.
     *
     * Use ODHOGDetector after training with this class. This is the training class for training HOG based detector. SVMlight is used here to train linear SVM on the HOG features. It supports the usage of multiple random windows in negetive training images
     * to increase the number of negetive features by the function 'setNOFeaturesNeg'. It also supports "Hard negetive" training which collects all the false positive
     * windows after initial training to retrain and obtain a new feature vector. Use the function 'setTrainHardNegetive' to enable this feature.
     *
     * \author Kripasindhu Sarkar
     */

    class ODHOGTrainer : public ODTrainer
    {

    public:

      ODHOGTrainer(const std::string & training_input_location_ = std::string(""), const std::string & trained_data_location_ = std::string(""), 
                   const cv::Size & win_size = cv::Size(64,128), const cv::Size & block_size = cv::Size(16,16), 
                   const cv::Size & block_stride = cv::Size(8,8), const cv::Size & cell_size = cv::Size(8,8), float hits_threshold = 0.0);

      int train();

      void init(){}

      const std::string & getPosSamplesDir() const;
      void setPosSamplesDir(const std::string & pos_samples_dir);

      const std::string & getNegSamplesDir() const;
      void setNegSamplesDir(const std::string & neg_samples_dir);

      int getNOFeaturesNeg() const;
      void setNOFeaturesNeg(int featno);

      const cv::Point & getStartHogPos() const;
      void setStartHogPos(const cv::Point & start_hog_pos);

      const cv::Size & getWinSize() const;
      void setWinSize(const cv::Size & win_size);

      const cv::Size & getBlockSize() const;
      void setBlockSize(const cv::Size & block_size);

      const cv::Size & getBlockStride() const;
      void setBlockStride(const cv::Size & block_stride);

      const cv::Size & getCellSize() const;
      void setCellSize(const cv::Size & cell_size);

      const cv::Size & getTrainingPadding() const;
      void setTrainingPadding(const cv::Size & training_padding);

      bool isTrainHardNegetive() const;
      void setTrainHardNegetive(bool train_hard_negative);

      double getHitThreshold() const;

    protected:
      //hog specific
      cv::Size win_size_;
      cv::Size block_size_;
      cv::Size block_stride_;
      cv::Size cell_size_;
      cv::Size win_stride_;
      cv::Size training_padding_;

      cv::HOGDescriptor hog_;

      //algo specific
      cv::Point start_hog_pos_;
      unsigned int no_features_neg_;
      bool train_hard_negative_;

      //directories
      std::string pos_samples_dir_, neg_samples_dir_;

      //properties retained
      double hit_threshold_;

    private:

      void readDescriptorsFromFile(const std::string & file_name, std::vector<float> & descriptor_vector);

      void save(const std::string & filename);

      void createHardTrainingData(const cv::HOGDescriptor & hog, double hit_threshold,
                                  const std::vector<std::string> & neg_file_names);

      void calculateFeaturesFromImageLoc(const cv::Mat & image_data, std::vector<float> & feature_vector,
                                         const cv::HOGDescriptor & hog, const cv::Point & start_pos);



      void detectTrainingSetTest(const cv::HOGDescriptor & hog, double hit_threshold,
                                 const std::vector<std::string> & pos_file_names, const std::vector<std::string> & neg_file_names);


      void calculateFeaturesFromInput(const std::string & image_filename, std::vector<float> & featureVector,
                                  cv::HOGDescriptor & hog);



      void saveDescriptorVectorToFile(const std::vector<float> & descriptor_vector, const std::string & file_name);

      void handleNegetivefile(const std::string & image_filename, cv::HOGDescriptor & hog, std::fstream & file);

      double trainWithSVMLight(const std::string & svm_model_file, const std::string & svm_descriptor_file, std::vector<float> & descriptor_vector);

      std::string features_file_;
      std::string svm_model_file_;
      std::string svm_model_hard_;
      std::string descriptor_vector_file_;
      std::string descriptor_vector_hard_;
      
    };

  }

}
