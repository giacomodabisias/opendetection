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

    ODCascadeDetector::ODCascadeDetector(const std::string & trained_data_location, double scale_factor, int min_neighbors, int flags, 
                                         const cv::Size & min_size, const cv::Size & max_size)
                                        : ODDetector2D(trained_data_location), scale_factor_(scale_factor), min_neighbors_(min_neighbors), 
                                                      min_size_(min_size), max_size_(max_size)
    {
      TRAINED_LOCATION_DENTIFIER_ = "CASCADE";
      TRAINED_DATA_ID_ = "cascade.xml";
      meta_info_ = true;
    }

    shared_ptr<ODDetections> ODCascadeDetector::detectOmni(shared_ptr<ODScene> scene)
    {
      std::cout << "not implemented, use detect()" <<std::endl; 
      return nullptr;
    };

    void ODCascadeDetector::init()
    {
      haar_cascade_ = make_shared<cv::CascadeClassifier>(fileutils::getFirstFile(getSpecificTrainingDataLocation(), TRAINED_DATA_ID_));
    }

    shared_ptr<ODDetections2D> ODCascadeDetector::detectOmni(shared_ptr<ODSceneImage> scene)
    {
      cv::Mat gray;
      cv::cvtColor(scene->getCVImage(), gray, CV_BGR2GRAY);
      // Find the faces in the frame:
      std::vector<cv::Rect_<int> > faces;
      haar_cascade_->detectMultiScale(gray, faces, scale_factor_, min_neighbors_, 0, min_size_, max_size_);

      //always create detections
      shared_ptr<ODDetections2D> detections = make_shared<ODDetections2D>();
      cv::Mat viz = scene->getCVImage().clone();
      cv::Rect face_i;
      for(size_t i = 0; i < faces.size(); ++i)
      {
        // Process face by face:
        face_i = faces[i];

        shared_ptr<ODDetection2D> detection2D = make_shared<ODDetection2D>(ODDetection::OD_DETECTION_CLASS, "FACE", 1);
        detection2D->setBoundingBox(face_i);
        detections->push_back(detection2D);

        if(meta_info_)
        {
          cv::rectangle(viz, face_i, CV_RGB(0, 255, 0), 1);
        }
      }
      detections->setMetainfoImage(viz);

      return detections;
    }

    shared_ptr<ODDetections> ODCascadeDetector::detect(shared_ptr<ODSceneImage> scene)
    {
      //always create detections
      shared_ptr<ODDetections> detections = make_shared<ODDetections>();

      cv::Mat gray;
      cv::cvtColor(scene->getCVImage(), gray, CV_BGR2GRAY);
      // Find the faces in the frame:
      std::vector<cv::Rect_<int> > faces;

      //hack for single detection,
      //note: maxsize = minsize = size of input image for single window detection
      //todo: implement in some other way of fast single detection; currently this will work, but maynot be fast
      haar_cascade_->detectMultiScale(gray, faces, 5, min_neighbors_, 0, gray.size(), gray.size());
      if(faces.size() > 0)
      {
        detections->push_back(make_shared<ODDetection>(ODDetection::OD_DETECTION_CLASS, "FACE", 1));
      }
      return detections;
    }

  }
}