#include "od/common/utils/ODViewer.h"

namespace od {

	ODViewer::ODViewer() : status_(UNDEFINED) {}

	template<typename PointT>
	void ODViewer::render(shared_ptr<pcl::PointCloud<PointT>> to_display, const std::string & cloud_name){
		if(status_ != POINTCLOUD){
			std::cout << "Switching viewer to PointCloud mode" << std::endl;
		}

		status_ = POINTCLOUD;

		if(!viewer_)
			viewer_ = make_shared<pcl::visualization::PCLVisualizer>(cloud_name);

		viewer_->addPointCloud<PointT>(to_display, cloud_name);

	}

	void ODViewer::render(const cv::Mat & to_display, const std::string & window_name){
		if(status_ != CVMAT){
			std::cout << "Switching viewer to cv::Mat mode" << std::endl;
		}

		status_ = CVMAT;

	}

	template<typename PointT>
	void ODViewer::render(pcl::PointCloud<PointT> & to_display, const std::string & cloud_name){

		if(status_ != POINTCLOUD){
			std::cout << "Switching viewer to PointCloud mode" << std::endl;
		}

		status_ = POINTCLOUD;

		if(!viewer_)
			viewer_ = make_shared<pcl::visualization::PCLVisualizer>(cloud_name);

		viewer_->addPointCloud<PointT>(to_display, cloud_name);

	}

	template<typename PointT>
	void ODViewer::update(shared_ptr<pcl::PointCloud<PointT>> to_display, const std::string & cloud_name){
		if(status_ != POINTCLOUD){
			std::cout << "No PointCloud to render! use render(shared_ptr<pcl::PointCloud<PointT>>) first!" << std::endl;
			return;
		}
	}

	void ODViewer::update(const cv::Mat & to_display, const std::string & window_name){
		if(status_ != CVMAT){
			std::cout << "No cv::Mat to render! use render(const cv::Mat & to_display) first!" << std::endl;
			return;
		}

	}

	template<typename PointT>
	void ODViewer::update(pcl::PointCloud<PointT> & to_display, const std::string & cloud_name){
		if(status_ != POINTCLOUD){
			std::cout << "No PointCloud to render! use render(shared_ptr<pcl::PointCloud<PointT>>) first!" << std::endl;
			return;
		}

	}

	void ODViewer::setColorHandler(const std::string & cloud_name){
		if(status_ != POINTCLOUD){
			std::cout << "No PointCloud to render! use render(shared_ptr<pcl::PointCloud<PointT>>) first!" << std::endl;
			return;
		}

	}

	void ODViewer::setBackGround(const cv::Scalar & color){
		setBackGround(color[2], color[1], color[0]);

	}

	void ODViewer::setBackGround(unsigned int r, unsigned int g, unsigned int b){
		switch(status_){
			case POINTCLOUD : 
				viewer_->setBackgroundColor(r, g, b);
				break;
			case CVMAT : 
				//img_ = color;
				break;
			case UNDEFINED : 
				std::cout << "Render al element first before adding a backgroud!" << std::endl;
				break;
		}

	}

}