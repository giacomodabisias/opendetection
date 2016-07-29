#pragma once


template <typename PointType>
ScenePointCloud<PointType>::ScenePointCloud(const shared_ptr<pcl::PointCloud<PointType> > point_cloud)
{
  point_cloud_ = point_cloud;
}

template <typename PointType>
ScenePointCloud<PointType>::ScenePointCloud(const std::string & point_cloud_file): 
                                                point_cloud_(new pcl::PointCloud<PointType>())
{
  if(pcl::io::loadPCDFile<PointType> (point_cloud_file, *point_cloud_ ) == -1)
  {
    std::cout << "ERROR: Couldn't read the file "<< point_cloud_file << std::endl;
  }
  path_ = point_cloud_file;
}

template <typename PointType>
shared_ptr<pcl::PointCloud<PointType> >  ScenePointCloud<PointType>::getPointCloud() const
{
  return point_cloud_;
}

template <typename PointType>
shared_ptr<pcl::PointCloud<PointType> > ScenePointCloud<PointType>::getPointCloudRef() const
{
  return point_cloud_;
}

template <typename PointType>
void ScenePointCloud<PointType>::setPointCloud(const shared_ptr<pcl::PointCloud<PointType> > point_cloud)
{
  point_cloud_ = point_cloud;
}

template <typename PointType>
void * ScenePointCloud<PointType>::getData() 
{ 
  return (void *)point_cloud_.get();
}