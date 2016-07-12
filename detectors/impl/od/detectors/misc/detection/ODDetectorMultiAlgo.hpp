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
#include "od/detectors/global2D/detection/ODCascadeDetector.h"
#include "od/detectors/global2D/detection/ODHOGDetector.h"
#include "od/detectors/global3D/detection/ODCADDetector3DGlobal.hpp"

namespace od
{

  template<typename PointT>
  class ODDetectorMultiAlgo2D : public ODDetector2D
  {
  public:
    ODDetectorMultiAlgo2D(const std::string & training_data_location_) : ODDetector2D(training_data_location_){}

    shared_ptr<ODDetections> detect(shared_ptr<ODSceneImage> scene);
    shared_ptr<ODDetections2D> detectOmni(shared_ptr<ODSceneImage> scene);

    shared_ptr<ODDetections> detect(shared_ptr<ODScenePointCloud<PointT> > scene);
    shared_ptr<ODDetections3D> detectOmni(shared_ptr<ODScenePointCloud<PointT> > scene);

    shared_ptr<ODDetections> detectOmni(shared_ptr<ODScene> scene);
    shared_ptr<ODDetections> detect(shared_ptr<ODScene> scene);

    void init();

  private:

    std::vector<shared_ptr<ODDetector2D> > detectors_2d_;
    std::vector<shared_ptr<ODDetector3D<PointT> > > detectors_3d_;

  };

  template<typename PointT>
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<PointT>::detectOmni(shared_ptr<ODScene> scene)
  {
    std::cout << "not implemented, use with shared_ptr<ODSceneImage> or shared_ptr<ODScenePointCloud<PointT>>" << std::endl; 
    return nullptr;
  }

  template<typename PointT>
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<PointT>::detect(shared_ptr<ODScene> scene)
  {
    std::cout << "not implemented, use with shared_ptr<ODSceneImage> or shared_ptr<ODScenePointCloud<PointT>>" << std::endl; 
    return nullptr;
  }

  //BASED ON 2D SCENE
  template<typename PointT>
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<PointT>::detect(shared_ptr<ODSceneImage> scene)
  {
    shared_ptr<ODDetections> detections_all = make_shared<ODDetections>();
    for(auto & d : detectors_2d_)
    {
      detections_all->append(d->detect(scene));
    }

    return detections_all;
  }

  template<typename PointT>
  shared_ptr<ODDetections2D> ODDetectorMultiAlgo2D<PointT>::detectOmni(shared_ptr<ODSceneImage > scene)
  {
    shared_ptr<ODDetections2D> detections_all = make_shared<ODDetections2D>();
    for(auto & d : detectors_2d_)
    {
      detections_all->append(d->detectOmni(scene));
    }

    return detections_all;
  }

  template<typename PointT>
  void ODDetectorMultiAlgo2D<PointT>::init()
  {
    //make a list of different algorithms
    //vector<ODDetector *> detectors = {new ODCascadeDetector(trained_data_location_), new ODHOGDetector(trained_data_location_), new ODCADRecognizer2DLocal(trained_data_location_)};
    
#ifdef WITH_BOOST_SHARED_PTR
      detectors_2d_.push_back(shared_ptr<g2d::ODCascadeDetector>(new g2d::ODCascadeDetector(trained_data_location_)));
      detectors_2d_.push_back(shared_ptr<g2d::ODHOGDetector>(new g2d::ODHOGDetector(trained_data_location_)));
#else
      detectors_2d_.push_back(make_shared<g2d::ODCascadeDetector>(trained_data_location_));
      detectors_2d_.push_back(make_shared<g2d::ODHOGDetector>(trained_data_location_));
#endif

    //  detectors.push_back(new ODCADRecognizer2DLocal(trained_data_location_));

    for(auto & d : detectors_2d_)
    {
      d->init();
    }
  }

#ifndef DOXYGEN_SHOULD_SKIP_THIS

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZ>::detectOmni(shared_ptr<ODScene> scene);

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZ>::detect(shared_ptr<ODScene> scene);

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZ>::detect(shared_ptr<ODSceneImage> scene);

  extern template
  shared_ptr<ODDetections2D> ODDetectorMultiAlgo2D<pcl::PointXYZ>::detectOmni(shared_ptr<ODSceneImage > scene);

  extern template
  void ODDetectorMultiAlgo2D<pcl::PointXYZ>::init();



  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGB>::detectOmni(shared_ptr<ODScene> scene);

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGB>::detect(shared_ptr<ODScene> scene);

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGB>::detect(shared_ptr<ODSceneImage> scene);

  extern template
  shared_ptr<ODDetections2D> ODDetectorMultiAlgo2D<pcl::PointXYZRGB>::detectOmni(shared_ptr<ODSceneImage > scene);

  extern template
  void ODDetectorMultiAlgo2D<pcl::PointXYZRGB>::init();




  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ODScene> scene);

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGBA>::detect(shared_ptr<ODScene> scene);

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGBA>::detect(shared_ptr<ODSceneImage> scene);

  extern template
  shared_ptr<ODDetections2D> ODDetectorMultiAlgo2D<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ODSceneImage > scene);

  extern template
  void ODDetectorMultiAlgo2D<pcl::PointXYZRGBA>::init();

#endif

  template<typename PointT>
  class ODDetectorMultiAlgo : public ODDetector
  {

  public:
    
    ODDetectorMultiAlgo(const std::string & training_data_location_) : ODDetector(training_data_location_){}

    shared_ptr<ODDetections> detect(shared_ptr<ODSceneImage> scene);
    shared_ptr<ODDetections2D> detectOmni(shared_ptr<ODSceneImage> scene);

    shared_ptr<ODDetections> detect(shared_ptr<ODScenePointCloud<PointT> > scene);
    shared_ptr<ODDetections3D> detectOmni(shared_ptr<ODScenePointCloud<PointT> > scene);

    shared_ptr<ODDetections> detectOmni(shared_ptr<ODScene> scene);
    shared_ptr<ODDetections> detect(shared_ptr<ODScene> scene);

    void init();

  private:

    std::vector<shared_ptr<ODDetector2D> > detectors_2d_;
    std::vector<shared_ptr<ODDetector3D<PointT> > > detectors_3d_;

  };

  template<typename PointT>
  shared_ptr<ODDetections> ODDetectorMultiAlgo<PointT>::detectOmni(shared_ptr<ODScene> scene)
  {
    std::cout << "not implemented, use with shared_ptr<ODSceneImage> or shared_ptr<ODScenePointCloud<PointT>>" << std::endl; 
    return nullptr;
  }

  template<typename PointT>
  shared_ptr<ODDetections> ODDetectorMultiAlgo<PointT>::detect(shared_ptr<ODScene> scene)
  {
    std::cout << "not implemented, use with shared_ptr<ODSceneImage> or shared_ptr<ODScenePointCloud<PointT>>" << std::endl; 
    return nullptr;
  }


  /////############BASED ON 3D SCENE#####################
  template<typename PointT>
  void ODDetectorMultiAlgo<PointT>::init()
  {
      //3D
#ifdef WITH_BOOST_SHARED_PTR
    detectors_3d_.push_back(shared_ptr<g3d::ODCADDetector3DGlobal<PointT> >(new g3d::ODCADDetector3DGlobal<PointT>(trained_data_location_, training_input_location_)));
#else
    detectors_3d_.push_back(make_shared<g3d::ODCADDetector3DGlobal<PointT> >(trained_data_location_, training_input_location_));
#endif
    for(auto & d : detectors_3d_)
    {
      d->init();
    }
  }

  template<typename PointT>
  shared_ptr<ODDetections> ODDetectorMultiAlgo<PointT>::detect(shared_ptr<ODScenePointCloud<PointT> > scene)
  {
    shared_ptr<ODDetections> detections_all = make_shared<ODDetections>();
    for(auto & d : detectors_3d_)
    {
      detections_all->append(d->detect(scene));
    }

    return detections_all;
  }

  template<typename PointT>
  shared_ptr<ODDetections3D> ODDetectorMultiAlgo<PointT>::detectOmni(shared_ptr<ODScenePointCloud<PointT> > scene)
  {
    shared_ptr<ODDetections3D> detections_all = make_shared<ODDetections3D>();
    for(auto & d : detectors_3d_)
    {
      detections_all->append(d->detectOmni(scene));
    }

    return detections_all;
  }


#ifndef DOXYGEN_SHOULD_SKIP_THIS


  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZ>::detectOmni(shared_ptr<ODScene> scene);

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZ>::detect(shared_ptr<ODScene> scene);

  extern template
  void ODDetectorMultiAlgo<pcl::PointXYZ>::init();

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZ>::detect(shared_ptr<ODScenePointCloud<pcl::PointXYZ> > scene);

  extern template
  shared_ptr<ODDetections3D> ODDetectorMultiAlgo<pcl::PointXYZ>::detectOmni(shared_ptr<ODScenePointCloud<pcl::PointXYZ> > scene);



  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGB>::detectOmni(shared_ptr<ODScene> scene);

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGB>::detect(shared_ptr<ODScene> scene);

  extern template
  void ODDetectorMultiAlgo<pcl::PointXYZRGB>::init();

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGB>::detect(shared_ptr<ODScenePointCloud<pcl::PointXYZRGB> > scene);

  extern template
  shared_ptr<ODDetections3D> ODDetectorMultiAlgo<pcl::PointXYZRGB>::detectOmni(shared_ptr<ODScenePointCloud<pcl::PointXYZRGB> > scene);



  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ODScene> scene);

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGBA>::detect(shared_ptr<ODScene> scene);

  extern template
  void ODDetectorMultiAlgo<pcl::PointXYZRGBA>::init();

  extern template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGBA>::detect(shared_ptr<ODScenePointCloud<pcl::PointXYZRGBA> > scene);

  extern template
  shared_ptr<ODDetections3D> ODDetectorMultiAlgo<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ODScenePointCloud<pcl::PointXYZRGBA> > scene);


#endif

}