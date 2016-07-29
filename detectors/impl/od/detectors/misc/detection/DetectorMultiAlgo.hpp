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
// Created by sarkar on 06.08.15.
//
#pragma once
#include "od/detectors/global2D/detection/CascadeDetector.h"
#include "od/detectors/global2D/detection/HOGDetector.h"
#include "od/detectors/global3D/detection/CADDetector3DGlobal.hpp"

namespace od
{

  template<typename PointT>
  class DetectorMultiAlgo2D : public Detector2D
  {
  public:
    DetectorMultiAlgo2D(const std::string & training_data_location_) : Detector2D(training_data_location_){}

    shared_ptr<Detections> detect(shared_ptr<SceneImage> scene);
    shared_ptr<Detections2D> detectOmni(shared_ptr<SceneImage> scene);

    shared_ptr<Detections> detect(shared_ptr<ScenePointCloud<PointT> > scene);
    shared_ptr<Detections3D> detectOmni(shared_ptr<ScenePointCloud<PointT> > scene);

    shared_ptr<Detections> detectOmni(shared_ptr<Scene> scene);
    shared_ptr<Detections> detect(shared_ptr<Scene> scene);

    void init();

  private:

    std::vector<shared_ptr<Detector2D> > detectors_2d_;
    std::vector<shared_ptr<Detector3D<PointT> > > detectors_3d_;

  };

  template<typename PointT>
  shared_ptr<Detections> DetectorMultiAlgo2D<PointT>::detectOmni(shared_ptr<Scene> scene)
  {
    std::cout << "not implemented, use with shared_ptr<SceneImage> or shared_ptr<ScenePointCloud<PointT>>" << std::endl; 
    return nullptr;
  }

  template<typename PointT>
  shared_ptr<Detections> DetectorMultiAlgo2D<PointT>::detect(shared_ptr<Scene> scene)
  {
    std::cout << "not implemented, use with shared_ptr<SceneImage> or shared_ptr<ScenePointCloud<PointT>>" << std::endl; 
    return nullptr;
  }

  //BASED ON 2D SCENE
  template<typename PointT>
  shared_ptr<Detections> DetectorMultiAlgo2D<PointT>::detect(shared_ptr<SceneImage> scene)
  {
    shared_ptr<Detections> detections_all = make_shared<Detections>();
    for(auto & d : detectors_2d_)
    {
      detections_all->append(d->detect(scene));
    }

    return detections_all;
  }

  template<typename PointT>
  shared_ptr<Detections2D> DetectorMultiAlgo2D<PointT>::detectOmni(shared_ptr<SceneImage > scene)
  {
    shared_ptr<Detections2D> detections_all = make_shared<Detections2D>();
    for(auto & d : detectors_2d_)
    {
      detections_all->append(d->detectOmni(scene));
    }

    return detections_all;
  }

  template<typename PointT>
  void DetectorMultiAlgo2D<PointT>::init()
  {
    //make a list of different algorithms
    //vector<Detector *> detectors = {new CascadeDetector(trained_data_location_), new HOGDetector(trained_data_location_), new CADRecognizer2DLocal(trained_data_location_)};
    
#ifndef WITH_BOOST_SHARED_PTR
    #if WITH_GPU
      detectors_2d_.push_back(shared_ptr<g2d::CascadeDetector>(new g2d::CascadeDetector(true, trained_data_location_)));
    #else
      detectors_2d_.push_back(shared_ptr<g2d::CascadeDetector>(new g2d::CascadeDetector(false, trained_data_location_)));
    #endif
      detectors_2d_.push_back(shared_ptr<g2d::HOGDetector>(new g2d::HOGDetector(trained_data_location_)));
#else
    #if WITH_GPU
      detectors_2d_.push_back(make_shared<g2d::CascadeDetector>(true, trained_data_location_));
    #else
      detectors_2d_.push_back(make_shared<g2d::CascadeDetector>(false, trained_data_location_));
    #endif
      detectors_2d_.push_back(make_shared<g2d::HOGDetector>(trained_data_location_));
#endif

    //  detectors.push_back(new CADRecognizer2DLocal(trained_data_location_));

    for(auto & d : detectors_2d_)
    {
      d->init();
    }
  }

#ifndef DOXYGEN_SHOULD_SKIP_THIS

  extern template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZ>::detectOmni(shared_ptr<Scene> scene);

  extern template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZ>::detect(shared_ptr<Scene> scene);

  extern template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZ>::detect(shared_ptr<SceneImage> scene);

  extern template
  shared_ptr<Detections2D> DetectorMultiAlgo2D<pcl::PointXYZ>::detectOmni(shared_ptr<SceneImage > scene);

  extern template
  void DetectorMultiAlgo2D<pcl::PointXYZ>::init();



  extern template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGB>::detectOmni(shared_ptr<Scene> scene);

  extern template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGB>::detect(shared_ptr<Scene> scene);

  extern template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGB>::detect(shared_ptr<SceneImage> scene);

  extern template
  shared_ptr<Detections2D> DetectorMultiAlgo2D<pcl::PointXYZRGB>::detectOmni(shared_ptr<SceneImage > scene);

  extern template
  void DetectorMultiAlgo2D<pcl::PointXYZRGB>::init();




  extern template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGBA>::detectOmni(shared_ptr<Scene> scene);

  extern template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGBA>::detect(shared_ptr<Scene> scene);

  extern template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGBA>::detect(shared_ptr<SceneImage> scene);

  extern template
  shared_ptr<Detections2D> DetectorMultiAlgo2D<pcl::PointXYZRGBA>::detectOmni(shared_ptr<SceneImage > scene);

  extern template
  void DetectorMultiAlgo2D<pcl::PointXYZRGBA>::init();

#endif

  template<typename PointT>
  class DetectorMultiAlgo : public Detector
  {

  public:
    
    DetectorMultiAlgo(const std::string & training_data_location_) : Detector(training_data_location_){}

    shared_ptr<Detections> detect(shared_ptr<SceneImage> scene);
    shared_ptr<Detections2D> detectOmni(shared_ptr<SceneImage> scene);

    shared_ptr<Detections> detect(shared_ptr<ScenePointCloud<PointT> > scene);
    shared_ptr<Detections3D> detectOmni(shared_ptr<ScenePointCloud<PointT> > scene);

    shared_ptr<Detections> detectOmni(shared_ptr<Scene> scene);
    shared_ptr<Detections> detect(shared_ptr<Scene> scene);

    void init();

  private:

    std::vector<shared_ptr<Detector2D> > detectors_2d_;
    std::vector<shared_ptr<Detector3D<PointT> > > detectors_3d_;

  };

  template<typename PointT>
  shared_ptr<Detections> DetectorMultiAlgo<PointT>::detectOmni(shared_ptr<Scene> scene)
  {
    std::cout << "not implemented, use with shared_ptr<SceneImage> or shared_ptr<ScenePointCloud<PointT>>" << std::endl; 
    return nullptr;
  }

  template<typename PointT>
  shared_ptr<Detections> DetectorMultiAlgo<PointT>::detect(shared_ptr<Scene> scene)
  {
    std::cout << "not implemented, use with shared_ptr<SceneImage> or shared_ptr<ScenePointCloud<PointT>>" << std::endl; 
    return nullptr;
  }


  /////############BASED ON 3D SCENE#####################
  template<typename PointT>
  void DetectorMultiAlgo<PointT>::init()
  {
      //3D
#ifndef WITH_BOOST_SHARED_PTR
    detectors_3d_.push_back(shared_ptr<g3d::CADDetector3DGlobal<PointT> >(new g3d::CADDetector3DGlobal<PointT>(trained_data_location_, training_input_location_)));
#else
    detectors_3d_.push_back(make_shared<g3d::CADDetector3DGlobal<PointT> >(trained_data_location_, training_input_location_));
#endif
    for(auto & d : detectors_3d_)
    {
      d->init();
    }
  }

  template<typename PointT>
  shared_ptr<Detections> DetectorMultiAlgo<PointT>::detect(shared_ptr<ScenePointCloud<PointT> > scene)
  {
    shared_ptr<Detections> detections_all = make_shared<Detections>();
    for(auto & d : detectors_3d_)
    {
      detections_all->append(d->detect(scene));
    }

    return detections_all;
  }

  template<typename PointT>
  shared_ptr<Detections3D> DetectorMultiAlgo<PointT>::detectOmni(shared_ptr<ScenePointCloud<PointT> > scene)
  {
    shared_ptr<Detections3D> detections_all = make_shared<Detections3D>();
    for(auto & d : detectors_3d_)
    {
      detections_all->append(d->detectOmni(scene));
    }

    return detections_all;
  }


#ifndef DOXYGEN_SHOULD_SKIP_THIS


  extern template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZ>::detectOmni(shared_ptr<Scene> scene);

  extern template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZ>::detect(shared_ptr<Scene> scene);

  extern template
  void DetectorMultiAlgo<pcl::PointXYZ>::init();

  extern template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZ>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZ> > scene);

  extern template
  shared_ptr<Detections3D> DetectorMultiAlgo<pcl::PointXYZ>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZ> > scene);



  extern template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGB>::detectOmni(shared_ptr<Scene> scene);

  extern template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGB>::detect(shared_ptr<Scene> scene);

  extern template
  void DetectorMultiAlgo<pcl::PointXYZRGB>::init();

  extern template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGB>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZRGB> > scene);

  extern template
  shared_ptr<Detections3D> DetectorMultiAlgo<pcl::PointXYZRGB>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZRGB> > scene);



  extern template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGBA>::detectOmni(shared_ptr<Scene> scene);

  extern template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGBA>::detect(shared_ptr<Scene> scene);

  extern template
  void DetectorMultiAlgo<pcl::PointXYZRGBA>::init();

  extern template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGBA>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZRGBA> > scene);

  extern template
  shared_ptr<Detections3D> DetectorMultiAlgo<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZRGBA> > scene);


#endif

}