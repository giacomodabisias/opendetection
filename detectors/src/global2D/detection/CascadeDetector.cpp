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
*///
// Created by sarkar on 17.07.15.
//

#include "od/detectors/global2D/detection/CascadeDetector.h"

namespace od
{

  namespace g2d
  {

    CascadeDetector::CascadeDetector(bool gpu, const std::string & trainer, const std::string & trained_data_location, double scale_factor, int min_neighbors, int flags, 
                                         const cv::Size & min_size, const cv::Size & max_size)
    {
      if(gpu){
        std::cout << "creating gpu cascade detector" << std::endl;
#if WITH_GPU
  #ifndef WITH_BOOST_SHARED_PTR
      cascade_detector_ = shared_ptr<od::gpu::g2d::CascadeDetectorImpl>(new od::gpu::g2d::CascadeDetectorImpl(trainer, trained_data_location, scale_factor, min_neighbors, flags, min_size, max_size));
  #else
      cascade_detector_ = make_shared<od::gpu::g2d::CascadeDetectorImpl>(trainer, trained_data_location, scale_factor, min_neighbors, flags, min_size, max_size);
  #endif
#else
        std::cout << "Error !! GPU CascadeDetector has not been compiled. Recompile with WITH_GPU." << std::endl;
        exit(-1);
#endif
      }
      else {
        std::cout << "creating cpu cascade detector" << std::endl;
#ifndef WITH_BOOST_SHARED_PTR
       cascade_detector_ = shared_ptr<CascadeDetectorImpl>(new CascadeDetectorImpl(trainer, trained_data_location, scale_factor, min_neighbors, flags, min_size, max_size));
#else
       cascade_detector_ = make_shared<CascadeDetectorImpl>(trainer, trained_data_location, scale_factor, min_neighbors, flags, min_size, max_size);
#endif    
      }
    }

    shared_ptr<Detections> CascadeDetector::detectOmni(shared_ptr<Scene> scene)
    {
      return cascade_detector_->detectOmni(scene);
    };

    void CascadeDetector::init()
    {
      cascade_detector_->init();
    }

    shared_ptr<Detections2D> CascadeDetector::detectOmni(shared_ptr<SceneImage> scene)
    {
      return cascade_detector_->detectOmni(scene);
    }

    shared_ptr<Detections> CascadeDetector::detect(shared_ptr<SceneImage> scene)
    {
      return cascade_detector_->detect(scene);
    }

    void CascadeDetector::setScale(const float scale)
    {
      cascade_detector_->setScale(scale);
    }

    float CascadeDetector::getScale() const
    {
      return cascade_detector_->getScale();
    }

    void CascadeDetector::setMinNeighbors(const unsigned int min_neighbors)
    {
      cascade_detector_->setMinNeighbors(min_neighbors);
    }

    unsigned int CascadeDetector::getMinNeighbors() const
    {
      return cascade_detector_->getMinNeighbors();
    }

    void CascadeDetector::setMinSize(const cv::Size & size)
    {
      cascade_detector_->setMinSize(size);
    }

    cv::Size CascadeDetector::getMinSize() const
    {
      return cascade_detector_->getMinSize();
    }

    void CascadeDetector::setMaxSize(const cv::Size & size)
    {
      cascade_detector_->setMaxSize(size);
    }

    cv::Size CascadeDetector::getMaxSize() const
    {
      return cascade_detector_->getMaxSize();
    }

  }
}