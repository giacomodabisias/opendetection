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
*/
//
// Created by sarkar on 10.06.15.
//

#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace od
{
  /** \brief Base class for Scenes. This contains information about the scenes. Scenes can be image scenes or point cloud scenes
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODScene
  {
  public:

    virtual void * getData() = 0;

    std::string const & getPath() const;

  protected:

    std::string path_;

  };

  /** \brief Class for Image Scene.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODSceneImage : public ODScene
  {
  public:

    ODSceneImage(const cv::Mat & cvimage);
    ODSceneImage(const std::string & path);

    const std::vector<cv::KeyPoint> & getKeypoints() const;
    void setKeypoints(const std::vector<cv::KeyPoint> & keypoints_);

    const cv::Mat & getDescriptors() const;
    void setDescriptors(const cv::Mat & descriptors_);

    cv::Mat getCVImage();

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
  class ODScenePointCloud : public ODScene
  {

  public:

    ODScenePointCloud(const typename pcl::PointCloud<PointType>::Ptr & point_cloud);
    ODScenePointCloud(const std::string & point_cloud_file);
    ODScenePointCloud(): point_cloud_(new typename pcl::PointCloud<PointType>()){}

    const typename pcl::PointCloud<PointType>::Ptr & getPointCloud() const;
    typename pcl::PointCloud<PointType>::Ptr & getPointCloudRef() const;
    void setPointCloud(const typename pcl::PointCloud<PointType>::Ptr & point_cloud_);

    void * getData();

  protected:

    typename pcl::PointCloud<PointType>::Ptr point_cloud_;

  };

  template <typename PointType>
  ODScenePointCloud<PointType>::ODScenePointCloud(const typename pcl::PointCloud<PointType>::Ptr & point_cloud)
  {
    point_cloud_ = point_cloud;
  }

  template <typename PointType>
  ODScenePointCloud<PointType>::ODScenePointCloud(const std::string & point_cloud_file): point_cloud_(new pcl::PointCloud<PointType>())
  {
    if(pcl::io::loadPCDFile<PointType> (point_cloud_file, *point_cloud_ ) == -1)
    {
      std::cout << "ERROR: Couldn't read the file "<< point_cloud_file << std::endl;
    }
    path_ = point_cloud_file;
  }

  template <typename PointType>
  const typename pcl::PointCloud<PointType>::Ptr & ODScenePointCloud<PointType>::getPointCloud() const
  {
    return point_cloud_;
  }

  template <typename PointType>
  typename pcl::PointCloud<PointType>::Ptr & ODScenePointCloud<PointType>::getPointCloudRef() const
  {
    return point_cloud_;
  }

  template <typename PointType>
  void ODScenePointCloud<PointType>::setPointCloud(const typename pcl::PointCloud<PointType>::Ptr & point_cloud)
  {
    point_cloud_ = point_cloud;
  }

  template <typename PointType>
  void * ODScenePointCloud<PointType>::getData() 
  { 
    return (void *)point_cloud_.get();
  }

}

