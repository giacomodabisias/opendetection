#pragma once
#include "od/common/utils/ODViewer.h"

namespace od {

	template<typename PointT>
	void ODViewer::render(shared_ptr<pcl::PointCloud<PointT> > to_display, const std::string & cloud_name, bool colored){
		if(status_ != POINTCLOUD){
			std::cout << "Switching viewer to PointCloud mode" << std::endl;
			if(status_ == CVMAT){
				cv::destroyWindow(mat_window_name_);
			}
		}

		status_ = POINTCLOUD;

		if(!viewer_ || pcl_window_name_ != cloud_name){
			viewer_ = make_shared<pcl::visualization::PCLVisualizer>(cloud_name);
			viewer_->setBackgroundColor(0, 0, 0);
		}

		pcl_window_name_ = cloud_name;

		if(colored)
		{
			pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(to_display);
			viewer_->addPointCloud<PointT>(to_display, rgb, cloud_name);
		}else{
			viewer_->addPointCloud<PointT>(to_display, cloud_name);
		}
	}

	template<typename PointT>
	void ODViewer::update(shared_ptr<pcl::PointCloud<PointT> > to_display, const std::string & cloud_name, bool colored){
		if(status_ != POINTCLOUD){
			std::cout << "No PointCloud to render! use render(shared_ptr<pcl::PointCloud<PointT>>) first!" << std::endl;
			return;
		}

		if(cloud_name != pcl_window_name_){
			std::cout << "No window " << cloud_name 
					  << " present. Please first use render(shared_ptr<pcl::PointCloud<PointT> >, const std::string & ) to create the new window" 
					  << std::endl;
			return;
		}

		if(colored)
		{
			pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(to_display);
			viewer_->updatePointCloud<PointT>(to_display, rgb, cloud_name);
		}else{
			viewer_->updatePointCloud<PointT>(to_display, cloud_name);
		}
	}

}