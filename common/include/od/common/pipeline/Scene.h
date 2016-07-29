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
*/
//
// Created by sarkar on 10.06.15.
//

#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "od/common/utils/Shared_pointers.h"

namespace od
{
  /** \brief Base class for Scenes. This contains information about the scenes. Scenes can be image scenes or point cloud scenes
   *
   * \author Kripasindhu Sarkar
   *
   */
  class Scene
  {
  public:

    virtual void * getData() = 0;

    const std::string & getPath() const;

  protected:

    std::string path_;

  };

  /** \brief Class for Image Scene.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class SceneImage : public Scene
  {
  public:

    SceneImage(const cv::Mat & cvimage);
    SceneImage(const std::string & path);

    const std::vector<cv::KeyPoint> & getKeypoints() const;
    void setKeypoints(const std::vector<cv::KeyPoint> & keypoints_);

    const cv::Mat & getDescriptors() const;
    void setDescriptors(const cv::Mat & descriptors_);

    cv::Mat getCVImage() const;

    void * getData();

  protected:

    cv::Mat cvimage_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
    bool is_trained_;

  };


  /** \brief Class for 3D scene containing point cloud.
    *
    * \author Kripasindhu Sarkar
    *
    */
  template <typename PointType>
  class ScenePointCloud : public Scene
  {

  public:

    ScenePointCloud(const shared_ptr<pcl::PointCloud<PointType> >  point_cloud);
    ScenePointCloud(const std::string & point_cloud_file);
    ScenePointCloud(): point_cloud_(new pcl::PointCloud<PointType>()){}

    shared_ptr<pcl::PointCloud<PointType> > getPointCloud() const;
    shared_ptr<pcl::PointCloud<PointType> > getPointCloudRef() const;
    void setPointCloud(const shared_ptr<pcl::PointCloud<PointType> > point_cloud_);

    void * getData();

  protected:

    shared_ptr<pcl::PointCloud<PointType> > point_cloud_;

  };

  #ifndef DOXYGEN_SHOULD_SKIP_THIS

    extern template
    ScenePointCloud<pcl::PointXYZ>::ScenePointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZ> > point_cloud);
    extern template
    ScenePointCloud<pcl::PointXYZRGB>::ScenePointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud);
    extern template 
    ScenePointCloud<pcl::PointXYZRGBA>::ScenePointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > point_cloud);

    extern template 
    ScenePointCloud<pcl::PointXYZ>::ScenePointCloud(const std::string & point_cloud_file);
    extern template
    ScenePointCloud<pcl::PointXYZRGB>::ScenePointCloud(const std::string & point_cloud_file);
    extern template 
    ScenePointCloud<pcl::PointXYZRGBA>::ScenePointCloud(const std::string & point_cloud_file);

    extern template 
    shared_ptr<pcl::PointCloud<pcl::PointXYZ> >  ScenePointCloud<pcl::PointXYZ>::getPointCloud() const;
    extern template 
    shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >  ScenePointCloud<pcl::PointXYZRGB>::getPointCloud() const;
    extern template 
    shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >  ScenePointCloud<pcl::PointXYZRGBA>::getPointCloud() const;

    extern template 
    shared_ptr<pcl::PointCloud<pcl::PointXYZ> > ScenePointCloud<pcl::PointXYZ>::getPointCloudRef() const;
    extern template 
    shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > ScenePointCloud<pcl::PointXYZRGB>::getPointCloudRef() const;
    extern template
    shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > ScenePointCloud<pcl::PointXYZRGBA>::getPointCloudRef() const;

    extern template 
    void ScenePointCloud<pcl::PointXYZ>::setPointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZ> > point_cloud);
    extern template 
    void ScenePointCloud<pcl::PointXYZRGB>::setPointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud);
    extern template 
    void ScenePointCloud<pcl::PointXYZRGBA>::setPointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > point_cloud);

    extern template 
    void * ScenePointCloud<pcl::PointXYZ>::getData();
    extern template 
    void * ScenePointCloud<pcl::PointXYZRGB>::getData();
    extern template 
    void * ScenePointCloud<pcl::PointXYZRGBA>::getData();

    #include "od/common/utils/Scene.hpp"

  #endif


}

