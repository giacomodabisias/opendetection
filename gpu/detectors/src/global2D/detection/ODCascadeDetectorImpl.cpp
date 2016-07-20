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

#include "od/gpu/detectors/global2D/detection/ODCascadeDetectorImpl.h"

namespace od
{

  namespace gpu 
  {

    namespace g2d
    {

      ODCascadeDetectorImpl::ODCascadeDetectorImpl(const std::string & trainer, const std::string & trained_data_location, double scale_factor, int min_neighbors, int flags, 
                                           const cv::Size & min_size, const cv::Size & max_size): 
                                           trained_data_location_(trained_data_location), scale_factor_(scale_factor), min_neighbors_(min_neighbors), 
                                           min_size_(min_size), max_size_(max_size), trained_data_id_(trainer), meta_info_(true), largest_(true)
      {}

      shared_ptr<ODDetections> ODCascadeDetectorImpl::detectOmni(shared_ptr<ODScene> scene)
      {
        std::cout << "not implemented, use detect()" <<std::endl; 
        return nullptr;
      }

      void ODCascadeDetectorImpl::init()
      {
        haar_cascade_ = cv::cuda::CascadeClassifier::create(trained_data_id_);
      }

      shared_ptr<ODDetections2D> ODCascadeDetectorImpl::detectOmni(shared_ptr<ODSceneImage> scene)
      {
        if(!haar_cascade_){
          std::cout << "Call init() first!" << std::endl;
          return nullptr;
        }

        cv::Mat gray;

        cv::Mat input = scene->getCVImage();
        objects_.clear();

        color_.upload(input);
        cv::cuda::cvtColor(color_, gray_, CV_BGR2GRAY);

        haar_cascade_->setFindLargestObject(largest_);
        haar_cascade_->setMaxObjectSize(max_size_);
        haar_cascade_->setMinNeighbors(min_neighbors_);
        haar_cascade_->setMinObjectSize(min_size_);

        haar_cascade_->setScaleFactor(scale_factor_);
        haar_cascade_->detectMultiScale(gray_, buf_);
        haar_cascade_->convert(buf_, objects_);

        //always create detections
        shared_ptr<ODDetections2D> detections = make_shared<ODDetections2D>();
        cv::Mat viz = scene->getCVImage().clone();
        for(auto & o : objects_)
        {
          // Process face by face:
          shared_ptr<ODDetection2D> detection2D = make_shared<ODDetection2D>(ODDetection::OD_CLASSIFICATION, "OBJ", 1);
          detection2D->setBoundingBox(o);
          detections->push_back(detection2D);

          if(meta_info_)
          {
            cv::rectangle(viz, o, CV_RGB(0, 255, 0), 1);
          }
        }
        detections->setMetainfoImage(viz);

        return detections;
      }

      shared_ptr<ODDetections> ODCascadeDetectorImpl::detect(shared_ptr<ODSceneImage> scene)
      {

        if(!haar_cascade_){
          std::cout << "Call init() first!" << std::endl;
          return nullptr;
        }

        cv::Mat gray;

        cv::Mat input = scene->getCVImage();
        objects_.clear();

        color_.upload(input);
        cv::cuda::cvtColor(color_, gray_, CV_BGR2GRAY);

        haar_cascade_->setFindLargestObject(largest_);
        haar_cascade_->setMaxObjectSize(gray.size());
        haar_cascade_->setMinNeighbors(min_neighbors_);
        haar_cascade_->setMinObjectSize(gray.size());

        haar_cascade_->setScaleFactor(scale_factor_);
        haar_cascade_->detectMultiScale(gray_, buf_);
        haar_cascade_->convert(buf_, objects_);

        //always create detections
        shared_ptr<ODDetections> detections = make_shared<ODDetections>();

        //hack for single detection,
        //note: maxsize = minsize = size of input image for single window detection
        //todo: implement in some other way of fast single detection; currently this will work, but maynot be fast
        if(objects_.size() > 0)
        {
          detections->push_back(make_shared<ODDetection>(ODDetection::OD_CLASSIFICATION, "OBJ", 1));
        }
        return detections;
      }

      void ODCascadeDetectorImpl::setScale(const float scale)
      {
        scale_factor_ = scale;
      }

      float ODCascadeDetectorImpl::getScale() const
      {
        return scale_factor_;
      }

      void ODCascadeDetectorImpl::setMinNeighbors(const unsigned int min_neighbors)
      {
        min_neighbors_ = min_neighbors;
      }

      unsigned int ODCascadeDetectorImpl::getMinNeighbors() const
      {
        return min_neighbors_;
      }

      void ODCascadeDetectorImpl::setMinSize(const cv::Size & size)
      {
        min_size_ = size;
      }

      cv::Size ODCascadeDetectorImpl::getMinSize() const
      {
        return min_size_;
      }

      void ODCascadeDetectorImpl::setMaxSize(const cv::Size & size)
      {
        max_size_ = size;
      }

      cv::Size ODCascadeDetectorImpl::getMaxSize() const
      {
        return max_size_;
      }

    }

  }

}