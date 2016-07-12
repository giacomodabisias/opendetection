#pragma once
#include "od/common/utils/ODViewer.h"

namespace od {

	template<typename PointT>
	void ODViewer::render(shared_ptr<pcl::PointCloud<PointT> > to_display, const std::string & cloud_name, bool colored)
	{
		if(status_ != POINTCLOUD){
			std::cout << "Switching viewer to PointCloud mode" << std::endl;
			if(status_ == CVMAT){
				cv::destroyWindow(mat_window_name_.c_str());
			}
		}

		status_ = POINTCLOUD;

		if(!viewer_ ){
#ifdef WITH_BOOST_SHARED_PTR
			viewer_ = shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(cloud_name));
#else
			viewer_ = make_shared<pcl::visualization::PCLVisualizer>(cloud_name);
#endif
			viewer_->setBackgroundColor(0, 0, 0);
		}

		clouds_.push_back(cloud_name);

		if(colored)
		{
			pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(to_display);
			viewer_->addPointCloud<PointT>(to_display, rgb, cloud_name);
		}else{
			viewer_->addPointCloud<PointT>(to_display, cloud_name);
		}
	}

	template<typename PointT>
	void ODViewer::render(shared_ptr<pcl::PointCloud<PointT> > to_display, 
		                  pcl::visualization::PointCloudColorHandlerRandom<PointT> & random_handler, const std::string & cloud_name)
	{
		if(status_ != POINTCLOUD){
			std::cout << "Switching viewer to PointCloud mode" << std::endl;
			if(status_ == CVMAT){
				cv::destroyWindow(mat_window_name_.c_str());
			}
		}

		status_ = POINTCLOUD;

		if(!viewer_ ){
#ifdef WITH_BOOST_SHARED_PTR
			viewer_ = shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(cloud_name));
#else
			viewer_ = make_shared<pcl::visualization::PCLVisualizer>(cloud_name);
#endif
			viewer_->setBackgroundColor(0, 0, 0);
		}

		clouds_.push_back(cloud_name);

		viewer_->addPointCloud<PointT>(to_display, random_handler, cloud_name);

	}

	template<typename PointT>
	void ODViewer::render(shared_ptr<ODScenePointCloud<PointT> > to_display, const std::string & cloud_name, bool colored)
	{
		render(to_display->getPointCloud(), cloud_name, colored);
	}

	template<typename PointT>
	void ODViewer::render(const ODScenePointCloud<PointT> & to_display, const std::string & cloud_name, bool colored)
	{
		render(to_display.getPointCloud(), cloud_name, colored);
	}

	template<typename PointT>
	void ODViewer::update(shared_ptr<pcl::PointCloud<PointT> > to_display, const std::string & cloud_name, bool colored)
	{
		if(status_ != POINTCLOUD){
			std::cout << "No PointCloud to render! use render(shared_ptr<pcl::PointCloud<PointT>>) first!" << std::endl;
			return;
		}


		if(!viewer_ || std::find(clouds_.begin(), clouds_.end(), cloud_name) != clouds_.end()){
			std::cout << "No cloud " << cloud_name 
					  << " present. Please first use render(shared_ptr<pcl::PointCloud<PointT> >, const std::string & ) to add the cloud" 
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

	template<typename PointT>
	void ODViewer::update(shared_ptr<ODScenePointCloud<PointT> > to_display, const std::string & cloud_name, bool colored)
	{
		update(to_display->getPointCloud(), cloud_name, colored);
	}

	template<typename PointT>
	void ODViewer::update(const ODScenePointCloud<PointT> & to_display, const std::string & cloud_name, bool colored)
	{
		update(to_display.getPointCloud(), cloud_name, colored);
	}

}