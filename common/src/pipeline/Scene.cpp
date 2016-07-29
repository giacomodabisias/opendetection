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

#include "od/common/pipeline/Scene.h"

namespace od {

	std::string const & Scene::getPath() const
	{
		return path_;
	}


	SceneImage::SceneImage(const cv::Mat & cvimage): is_trained_(false)
	{
		cvimage_ = cvimage.clone();
	}

	SceneImage::SceneImage(const std::string & path): is_trained_(false)
	{
		cvimage_ = cv::imread(path);
		path_ = path;
	}

	const std::vector<cv::KeyPoint> & SceneImage::getKeypoints() const
	{
		return keypoints_;
	}

	void SceneImage::setKeypoints(const std::vector<cv::KeyPoint> & keypoints)
	{
		keypoints_ = keypoints;
	}

	const cv::Mat & SceneImage::getDescriptors() const
	{
		return descriptors_;
	}

	void SceneImage::setDescriptors(const cv::Mat & descriptors)
	{
		descriptors_ = descriptors;
		is_trained_ = true;
	}

	cv::Mat SceneImage::getCVImage() const
	{
		return cvimage_;
	}

	void * SceneImage::getData()
	{
		return &cvimage_;
	}


  template
  ScenePointCloud<pcl::PointXYZ>::ScenePointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZ> > point_cloud);
  template
  ScenePointCloud<pcl::PointXYZRGB>::ScenePointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud);
  template 
  ScenePointCloud<pcl::PointXYZRGBA>::ScenePointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > point_cloud);

  template 
  ScenePointCloud<pcl::PointXYZ>::ScenePointCloud(const std::string & point_cloud_file);
  template
  ScenePointCloud<pcl::PointXYZRGB>::ScenePointCloud(const std::string & point_cloud_file);
  template 
  ScenePointCloud<pcl::PointXYZRGBA>::ScenePointCloud(const std::string & point_cloud_file);

  template 
  shared_ptr<pcl::PointCloud<pcl::PointXYZ> >  ScenePointCloud<pcl::PointXYZ>::getPointCloud() const;
  template 
  shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >  ScenePointCloud<pcl::PointXYZRGB>::getPointCloud() const;
  template 
  shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >  ScenePointCloud<pcl::PointXYZRGBA>::getPointCloud() const;

  template 
  shared_ptr<pcl::PointCloud<pcl::PointXYZ> > ScenePointCloud<pcl::PointXYZ>::getPointCloudRef() const;
  template 
  shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > ScenePointCloud<pcl::PointXYZRGB>::getPointCloudRef() const;
  template
  shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > ScenePointCloud<pcl::PointXYZRGBA>::getPointCloudRef() const;

  template 
  void ScenePointCloud<pcl::PointXYZ>::setPointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZ> > point_cloud);
  template 
  void ScenePointCloud<pcl::PointXYZRGB>::setPointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud);
  template 
  void ScenePointCloud<pcl::PointXYZRGBA>::setPointCloud(const shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > point_cloud);

  template 
  void * ScenePointCloud<pcl::PointXYZ>::getData();
  template 
  void * ScenePointCloud<pcl::PointXYZRGB>::getData();
  template 
  void * ScenePointCloud<pcl::PointXYZRGBA>::getData();


}