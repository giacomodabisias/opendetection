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
// Created by sarkar on 06.08.15.
//
#pragma once
#include "od/common/pipeline/ODDetector.h"
#include "od/detectors/global2D/detection/ODCascadeDetector.h"
#include "od/detectors/global2D/detection/ODHOGDetector.h"
#include "od/detectors/global2D/ODFaceRecognizer.h"
#include "od/detectors/local2D/detection/ODCADRecognizer2DLocal.h"
//3D detectors
#include "od/detectors/global3D/detection/ODCADDetector3DGlobal.hpp"

namespace od
{

  template<typename PointT>
  class ODDetectorMultiAlgo2D : public ODDetector2D
  {
  public:
    ODDetectorMultiAlgo2D(const std::string & training_data_location_) : ODDetector2D(training_data_location_){}

    shared_ptr<ODDetections> detect(shared_ptr<ODSceneImage> scene) ;
    shared_ptr<ODDetections2D> detectOmni(shared_ptr<ODSceneImage> scene);

    shared_ptr<ODDetections> detect(shared_ptr<ODScenePointCloud<PointT> > scene);
    shared_ptr<ODDetections3D> detectOmni(shared_ptr<ODScenePointCloud<PointT> > scene);


    void init();

  private:

    std::vector<shared_ptr<ODDetector2D> > detectors_2d_;
    std::vector<shared_ptr<ODDetector3D<PointT> > > detectors_3d_;

  };
  template<typename PointT>
  class ODDetectorMultiAlgo : public ODDetector
  {

  public:
    ODDetectorMultiAlgo(const std::string & training_data_location_) : ODDetector(training_data_location_){}

    shared_ptr<ODDetections> detect(shared_ptr<ODSceneImage > scene) ;
    shared_ptr<ODDetections2D> detectOmni(shared_ptr<ODSceneImage > scene);

    shared_ptr<ODDetections> detect(shared_ptr<ODScenePointCloud<PointT> > scene);
    shared_ptr<ODDetections3D> detectOmni(shared_ptr<ODScenePointCloud<PointT> > scene);

    void init();

  private:
    std::vector<od::shared_ptr<ODDetector2D> > detectors_2d_;
    std::vector<shared_ptr<ODDetector3D<PointT> > > detectors_3d_;
  };


  //BASED ON 2D SCENE
  template<typename PointT>
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<PointT>::detect(shared_ptr<ODSceneImage > scene)
  {
    shared_ptr<ODDetections> detections_all = make_shared<ODDetections>();
    for (size_t i = 0; i < detectors_2d_.size(); ++i)
    {
      shared_ptr<ODDetections> detections_individual = detectors_2d_[i]->detect(scene);
      detections_all->append(detections_individual);
    }

    return detections_all;
  }

  template<typename PointT>
  shared_ptr<ODDetections2D> ODDetectorMultiAlgo2D<PointT>::detectOmni(shared_ptr<ODSceneImage > scene)
  {
    shared_ptr<ODDetections2D> detections_all = make_shared<ODDetections2D>();
    for (size_t i = 0; i < detectors_2d_.size(); ++i)
    {
      shared_ptr<ODDetections2D> detections_individual = detectors_2d_[i]->detectOmni(scene);
      detections_all->append(detections_individual);
    }

    return detections_all;
  }

  template<typename PointT>
  void ODDetectorMultiAlgo2D<PointT>::init()
  {
    //make a list of different algorithms
    //vector<ODDetector *> detectors = {new ODCascadeDetector(trained_data_location_), new ODHOGDetector(trained_data_location_), new ODCADRecognizer2DLocal(trained_data_location_)};
    detectors_2d_.push_back(new g2d::ODCascadeDetector(trained_data_location_));
    detectors_2d_.push_back(new g2d::ODHOGDetector(trained_data_location_));
    //  detectors.push_back(new ODCADRecognizer2DLocal(trained_data_location_));

    for(size_t i = 0; i < detectors_2d_.size(); ++i)
    {
      detectors_2d_[i]->init();
    }
  }

  /////############BASED ON 3D SCENE#####################
  template<typename PointT>
  void ODDetectorMultiAlgo<PointT>::init()
  {
      //3D
    detectors_3d_.push_back(new g3d::ODCADDetector3DGlobal<PointT>(trained_data_location_, training_input_location_));
    for(size_t i = 0; i < detectors_3d_.size(); ++i)
    {
      detectors_3d_[i]->init();
    }
  }

  template<typename PointT>
  shared_ptr<ODDetections> ODDetectorMultiAlgo<PointT>::detect(shared_ptr<ODScenePointCloud<PointT> > scene)
  {
    shared_ptr<ODDetections> detections_all = make_shared<ODDetections>();
    for(size_t i = 0; i < detectors_3d_.size(); ++i)
    {
      shared_ptr<ODDetections> detections_individual = detectors_3d_[i]->detect(scene);
      detections_all->append(detections_individual);
    }

    return detections_all;
  }

  template<typename PointT>
  shared_ptr<ODDetections3D> ODDetectorMultiAlgo<PointT>::detectOmni(shared_ptr<ODScenePointCloud<PointT> > scene)
  {
    shared_ptr<ODDetections3D> detections_all = make_shared<ODDetections3D>;
    for(size_t i = 0; i < detectors_3d_.size(); i++)
    {
      shared_ptr<ODDetections3D> detections_individual = detectors_3d_[i]->detectOmni(scene);
      detections_all->append(detections_individual);
    }

    return detections_all;
  }

}