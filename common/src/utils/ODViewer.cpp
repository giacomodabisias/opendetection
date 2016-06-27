#include "od/common/utils/ODViewer.h"
#if(WIN32)
	#include <Windows.h>
#endif

namespace od {

	ODViewer::ODViewer() : status_(UNDEFINED) {}

	void ODViewer::render(const cv::Mat & to_display, const std::string & window_name){
		if(status_ != CVMAT){
			std::cout << "Switching viewer to cv::Mat mode" << std::endl;
			if(status_ == POINTCLOUD){
				//BUGGED IN PCL, its not closing https://github.com/PointCloudLibrary/pcl/issues/172
				//viewer_->close();
				//Alternative solution
#if(WIN32)
				HWND hWnd = (HWND)viewer3d->getRenderWindow()->GetGenericWindowId(); 
				viewer_.reset();
				DestroyWindow(hWnd); 
#else
				auto p = viewer_->getRenderWindow(); 
				viewer_.reset();
				p->Finalize();
#endif
			}
		}
		status_ = CVMAT;
		mat_window_name_ = window_name;
		mat_ = make_shared<cv::Mat>(to_display);
		cvStartWindowThread();
		cvNamedWindow(mat_window_name_.c_str(), CV_WINDOW_AUTOSIZE);
	}

	void ODViewer::update(const cv::Mat & to_display, const std::string & window_name){
		if(status_ != CVMAT){
			std::cout << "No cv::Mat to render! use render(const cv::Mat &) first!" << std::endl;
			return;
		}
		if(window_name != mat_window_name_){
			std::cout << "No window " << window_name << " present. Please first use render(const cv::Mat &, const std::string &) to create the new window" << std::endl;
			return;
		}

		mat_ = make_shared<cv::Mat>(to_display);

	}

	void ODViewer::setBackGround(const cv::Scalar & color){
		switch(status_){
			case POINTCLOUD : 
				viewer_->setBackgroundColor(color[2], color[1], color[0]);
				break;
			case CVMAT : 
				*mat_ = color;
				break;
			case UNDEFINED : 
				std::cout << "Render an element first before adding a backgroud!" << std::endl;
				break;
		}

	}

	bool ODViewer::toStop(){

		switch(status_){
			case POINTCLOUD : 
				return(viewer_->wasStopped());
				break;
			case CVMAT :
				return (cv::waitKey(50) == 27 || cv::waitKey(50) == 'q');
				break;
			case UNDEFINED : 
				break;
		}
		return false;
	
	}

	void ODViewer::setBackGround(unsigned int r, unsigned int g, unsigned int b){
		setBackGround(cv::Scalar(b, g, r));
	}

	void ODViewer::spin(){
		switch(status_){
			case POINTCLOUD : 
				if(viewer_)
					viewer_->spinOnce();
				break;
			case CVMAT : 
				cv::imshow(mat_window_name_, *mat_);
				cv::waitKey(10);
				break;
			case UNDEFINED : 
				std::cout << "Nothing to render!" << std::endl;
				break;
		}
	}

	// Explicit template function instantiation 
	template void ODViewer::render<pcl::PointXYZ>(shared_ptr<pcl::PointCloud<pcl::PointXYZ> >, const std::string &, bool);
	template void ODViewer::render<pcl::PointXYZRGB>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >, const std::string &, bool);
	template void ODViewer::render<pcl::PointXYZRGBA>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >, const std::string &, bool);

	template void ODViewer::update<pcl::PointXYZ>(shared_ptr<pcl::PointCloud<pcl::PointXYZ> >, const std::string &, bool);
	template void ODViewer::update<pcl::PointXYZRGB>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >, const std::string &, bool);
	template void ODViewer::update<pcl::PointXYZRGBA>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >, const std::string &, bool);
}