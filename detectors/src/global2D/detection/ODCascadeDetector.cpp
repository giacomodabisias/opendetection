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

#include "od/detectors/global2D/detection/ODCascadeDetector.h"

namespace od
{

  namespace g2d
  {

    ODCascadeDetector::ODCascadeDetector(const std::string & trainer, const std::string & trained_data_location, double scale_factor, int min_neighbors, int flags, 
                                         const cv::Size & min_size, const cv::Size & max_size): 
                                         ODDetector2D(trained_data_location), scale_factor_(scale_factor), min_neighbors_(min_neighbors), 
                                            min_size_(min_size), max_size_(max_size)
    {
      trained_location_identifier_ = std::string("CASCADE");
      trained_data_id_ = trainer;
      meta_info_ = true;
    }

    shared_ptr<ODDetections> ODCascadeDetector::detectOmni(shared_ptr<ODScene> scene)
    {
      std::cout << "not implemented, use detect()" <<std::endl; 
      return nullptr;
    };

    void ODCascadeDetector::init()
    {
#ifdef WITH_BOOST_SHARED_PTR
      haar_cascade_ = shared_ptr<cv::CascadeClassifier>(new cv::CascadeClassifier(trained_data_id_));
#else
      haar_cascade_ = make_shared<cv::CascadeClassifier>(trained_data_id_);
#endif
    }

    shared_ptr<ODDetections2D> ODCascadeDetector::detectOmni(shared_ptr<ODSceneImage> scene)
    {

      if(!haar_cascade_){
        std::cout << "Call init() first!" << std::endl;
        return nullptr;
      }

      cv::Mat gray;
      cv::cvtColor(scene->getCVImage(), gray, CV_BGR2GRAY);

      std::vector<cv::Rect_<int> > objects;
      haar_cascade_->detectMultiScale(gray, objects, scale_factor_, min_neighbors_, 0, min_size_, max_size_);

      //always create detections
      shared_ptr<ODDetections2D> detections = make_shared<ODDetections2D>();
      cv::Mat viz = scene->getCVImage();
      for(auto & o : objects)
      {
        // Process object by object:
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

    shared_ptr<ODDetections> ODCascadeDetector::detect(shared_ptr<ODSceneImage> scene)
    {

      if(!haar_cascade_){
        std::cout << "Call init() first!" << std::endl;
        return nullptr;
      }

      //always create detections
      shared_ptr<ODDetections> detections = make_shared<ODDetections>();

      cv::Mat gray;
      cv::cvtColor(scene->getCVImage(), gray, CV_BGR2GRAY);
      // Find the objects in the frame:
      std::vector<cv::Rect_<int> > objects;

      //hack for single detection,
      //note: maxsize = minsize = size of input image for single window detection
      //todo: implement in some other way of fast single detection; currently this will work, but maynot be fast
      haar_cascade_->detectMultiScale(gray, objects, 5, min_neighbors_, 0, gray.size(), gray.size());
      if(objects.size() > 0)
      {
        detections->push_back(make_shared<ODDetection>(ODDetection::OD_CLASSIFICATION, "OBJ", 1));
      }
      return detections;
    }

    void ODCascadeDetector::setScale(const float scale)
    {
      scale_factor_ = scale;
    }

    float ODCascadeDetector::getScale() const
    {
      return scale_factor_;
    }

    void ODCascadeDetector::setMinNeighbors(const unsigned int min_neighbors)
    {
      min_neighbors_ = min_neighbors;
    }

    unsigned int ODCascadeDetector::getMinNeighbors() const
    {
      return min_neighbors_;
    }

    void ODCascadeDetector::setMinSize(const cv::Size & size)
    {
      min_size_ = size;
    }

    cv::Size ODCascadeDetector::getMinSize() const
    {
      return min_size_;
    }

    void ODCascadeDetector::setMaxSize(const cv::Size & size)
    {
      max_size_ = size;
    }

    cv::Size ODCascadeDetector::getMaxSize() const
    {
      return max_size_;
    }

  }
}