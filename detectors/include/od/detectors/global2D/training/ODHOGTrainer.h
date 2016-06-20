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

      ODHOGTrainer(const std::string & training_input_location_ = "", const std::string & trained_data_location_ = "", 
                   const cv::Size & win_size = cv::Size(64,128), const cv::Size & block_size = cv::Size(16,16), 
                   const cv::Size & block_stride = cv::Size(8,8), const cv::Size & cell_size = cv::Size(8,8), float hits_threshold = 0.0);

      int train();

      void init(){}

      const std::string & getPosSamplesDir() const
      {
        return pos_samples_dir_;
      }

      void setPosSamplesDir(const std::string & pos_samples_dir)
      {
        pos_samples_dir_ = pos_samples_dir;
      }

      const std::string & getNegSamplesDir() const
      {
        return neg_samples_dir_;
      }

      void setNegSamplesDir(const std::string & neg_samples_dir)
      {
        neg_samples_dir_ = neg_samples_dir;
      }

      int getNOFeaturesNeg() const
      {
        return no_features_neg_;
      }

      void setNOFeaturesNeg(int featno)
      {
        no_features_neg_ = featno;
      }

      const cv::Point & getStartHogPos() const
      {
        return start_hog_pos_;
      }

      void setStartHogPos(const cv::Point & start_hog_pos)
      {
        start_hog_pos_ = start_hog_pos;
      }

      const cv::Size & getWinSize() const
      {
        return win_size_;
      }

      void setWinSize(const cv::Size & win_size)
      {
        win_size_ = win_size;
      }

      const cv::Size & getBlockSize() const
      {
        return block_size_;
      }

      void setBlockSize(const cv::Size & block_size)
      {
        block_size_ = block_size;
      }

      const cv::Size & getBlockStride() const
      {
        return block_stride_;
      }

      void setBlockStride(const cv::Size & block_stride)
      {
        block_stride_ = block_stride;
      }

      const cv::Size & getCellSize() const
      {
        return cell_size_;
      }

      void setCellSize(const cv::Size & cell_size)
      {
        cell_size_ = cell_size;
      }

      const cv::Size & getTrainingPadding() const
      {
        return training_padding_;
      }

      void setTrainingPadding(const cv::Size & training_padding)
      {
        training_padding_ = training_padding;
      }

      bool isTrainHardNegetive() const
      {
        return train_hard_negative_;
      }

      void setTrainHardNegetive(bool train_hard_negative)
      {
        train_hard_negative_ = train_hard_negative;
      }

      double getHitThreshold() const
      {
        return hit_threshold_;
      }

    protected:
      //hog specific
      cv::Size win_size_;
      cv::Size block_size_;
      cv::Size block_stride_;
      cv::Size cell_size_;

      cv::HOGDescriptor hog_;

      //algo specific
      cv::Size training_padding_;
      cv::Point start_hog_pos_;
      unsigned int no_features_neg_;
      cv::Size win_stride_;
      bool train_hard_negative_;

      //directories
      std::string pos_samples_dir_;
      std::string neg_samples_dir_;

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
