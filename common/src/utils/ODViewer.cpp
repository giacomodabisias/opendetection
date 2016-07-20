#include "od/common/utils/ODViewer.h"
#if(WIN32)
	#include <Windows.h>
#endif

namespace od {

	ODViewer::ODViewer() : status_(UNDEFINED) {}

	void ODViewer::render(const cv::Mat & to_display, const std::string & window_name)
	{
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
		//cvStartWindowThread();
		cvNamedWindow(mat_window_name_.c_str(), CV_WINDOW_AUTOSIZE);
	}

	void ODViewer::render(const ODSceneImage & to_display, const std::string & window_name)
	{
		render(to_display.getCVImage(), window_name);
	}

	void ODViewer::render(shared_ptr<ODSceneImage> to_display, const std::string & window_name)
	{
		render(to_display->getCVImage(), window_name);
	}

	void ODViewer::initCVWindow(const std::string & window_name)
	{
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
		mat_ = make_shared<cv::Mat>();
		//cvStartWindowThread();
		cvNamedWindow(mat_window_name_.c_str(), CV_WINDOW_AUTOSIZE);
	}

	void ODViewer::initPCLWindow(const std::string & window_name)
	{
		if(status_ != POINTCLOUD){
			std::cout << "Switching viewer to PointCloud mode" << std::endl;
			if(status_ == CVMAT){
				cv::destroyWindow(mat_window_name_.c_str());
			}
		}

		status_ = POINTCLOUD;

		if(!viewer_ || pcl_window_name_ != window_name)
		{
#ifdef WITH_BOOST_SHARED_PTR
			viewer_ = shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(window_name));
#else
			viewer_ = make_shared<pcl::visualization::PCLVisualizer>(window_name);
#endif
			viewer_->setBackgroundColor(0, 0, 0);
		}

		pcl_window_name_ = window_name;
	}

	void ODViewer::update(const cv::Mat & to_display, const std::string & window_name)
	{
		if(status_ != CVMAT)
		{
			std::cout << "No cv::Mat to render! use render(const cv::Mat &) first!" << std::endl;
			return;
		}
		if(window_name != mat_window_name_)
		{
			render(to_display, window_name);
			return;
		}

		mat_ = make_shared<cv::Mat>(to_display);

	}

	void ODViewer::update(const ODSceneImage & to_display, const std::string & window_name)
	{
		update(to_display.getCVImage(), window_name);
	}
	void ODViewer::update(shared_ptr<ODSceneImage> to_display, const std::string & window_name)
	{
		update(to_display->getCVImage(), window_name);
	}

	void ODViewer::setBackGround(const cv::Scalar & color)
	{
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

	bool ODViewer::toStop()
	{

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

	void ODViewer::setBackGround(unsigned int r, unsigned int g, unsigned int b)
	{
		setBackGround(cv::Scalar(b, g, r));
	}

	void ODViewer::spin()
	{
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

	unsigned int ODViewer::wait(unsigned int time) const
	{
		return cv::waitKey(time);
	}

	void ODViewer::addText(const std::string & text, pcl::PointXYZ pos, double textScale, cv::Scalar color)
	{
		if(status_ == CVMAT)
		{
			cv::putText(*mat_, text, cv::Point(pos.x, pos.y), cv::FONT_HERSHEY_SIMPLEX, textScale, color);
		}
		else if(status_ == POINTCLOUD)
		{
			viewer_->addText3D(text, pos, textScale, color[2], color[1], color[0]);
		
		}
	}

	void ODViewer::addText(const std::string & text, cv::Point3f pos, double textScale, cv::Scalar color)
	{
		if(status_ == CVMAT)
		{
			cv::putText(*mat_, text, cv::Point(pos.x, pos.y), cv::FONT_HERSHEY_SIMPLEX, textScale, color);
		}
		else if(status_ == POINTCLOUD)
		{
			viewer_->addText3D(text, pcl::PointXYZ(pos.x, pos.y, pos.z), textScale, color[2], color[1], color[0]);
		
		}
	}

	void ODViewer::remove(const std::string & text)
	{
		if(status_ == CVMAT)
		{	
			if(text != mat_window_name_)
			{
				std::cout << "no window " << text << " to remove" << std::endl;
				return;
			}
			cv::destroyWindow(mat_window_name_.c_str());
			mat_window_name_ = std::string();
		}
		else if(status_ == POINTCLOUD)
		{
			if(std::remove(clouds_.begin(), clouds_.end(), text) == clouds_.end())
			{
				std::cout << "no cloud " << text << " to remove" << std::endl;
				return;
			}
			viewer_->removePointCloud(text);
		}
	}

	void ODViewer::removeAll()
	{
		if(status_ == CVMAT)
		{
			remove(mat_window_name_);
		}
		else if(status_ == POINTCLOUD)
		{
			viewer_->removeAllPointClouds();
			clouds_.clear();	
		}
	}

	void ODViewer::removeAllShapes()
	{
		if(status_ == CVMAT)
		{
			std::cout << "Not in pcl mode!" << std::endl;
		}
		else if(status_ == POINTCLOUD)
		{
			viewer_->removeAllShapes();
		}

	}


	void ODViewer::removeText(const std::string & text)
	{
		if(status_ == CVMAT)
		{
			std::cout << "Not in pcl mode!" << std::endl;
		}
		else if(status_ == POINTCLOUD)
		{
			viewer_->removeText3D(text);
		}

	}

	void ODViewer::removeShape(const std::string & text)
	{
		if(status_ == CVMAT)
		{
			std::cout << "Not in pcl mode!" << std::endl;
		}
		else if(status_ == POINTCLOUD)
		{
			viewer_->removeShape(text);
		}

	}

	shared_ptr<pcl::visualization::PCLVisualizer> ODViewer::getViewer()
	{
		return viewer_;
	}



	// Explicit template function instantiation 
	template void ODViewer::render<pcl::PointXYZ>(shared_ptr<pcl::PointCloud<pcl::PointXYZ> >, const std::string &, bool);
	template void ODViewer::render<pcl::PointXYZRGB>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >, const std::string &, bool);
	template void ODViewer::render<pcl::PointXYZRGBA>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >, const std::string &, bool);

	template void ODViewer::render<pcl::PointXYZ>(shared_ptr<pcl::PointCloud<pcl::PointXYZ> > to_display, 
				    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> & random_handler, const std::string & cloud_name);
	template void ODViewer::render<pcl::PointXYZRGB>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > to_display, 
				    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> & random_handler, const std::string & cloud_name);
	template void ODViewer::render<pcl::PointXYZRGBA>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > to_display, 
				    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGBA> & random_handler, const std::string & cloud_name);

	template void ODViewer::render<pcl::PointXYZ>(shared_ptr<ODScenePointCloud<pcl::PointXYZ> >, const std::string &, bool);
	template void ODViewer::render<pcl::PointXYZRGB>(shared_ptr<ODScenePointCloud<pcl::PointXYZRGB> >, const std::string &, bool);
	template void ODViewer::render<pcl::PointXYZRGBA>(shared_ptr<ODScenePointCloud<pcl::PointXYZRGBA> >, const std::string &, bool);

	template void ODViewer::render<pcl::PointXYZ>(const ODScenePointCloud<pcl::PointXYZ> &, const std::string &, bool);
	template void ODViewer::render<pcl::PointXYZRGB>(const ODScenePointCloud<pcl::PointXYZRGB> &, const std::string &, bool);
	template void ODViewer::render<pcl::PointXYZRGBA>(const ODScenePointCloud<pcl::PointXYZRGBA> &, const std::string &, bool);

	template void ODViewer::update<pcl::PointXYZ>(shared_ptr<pcl::PointCloud<pcl::PointXYZ> >, const std::string &, bool);
	template void ODViewer::update<pcl::PointXYZRGB>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >, const std::string &, bool);
	template void ODViewer::update<pcl::PointXYZRGBA>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >, const std::string &, bool);

	template void ODViewer::update<pcl::PointXYZ>(shared_ptr<ODScenePointCloud<pcl::PointXYZ> >, const std::string &, bool);
	template void ODViewer::update<pcl::PointXYZRGB>(shared_ptr<ODScenePointCloud<pcl::PointXYZRGB> >, const std::string &, bool);
	template void ODViewer::update<pcl::PointXYZRGBA>(shared_ptr<ODScenePointCloud<pcl::PointXYZRGBA> >, const std::string &, bool);

	template void ODViewer::update<pcl::PointXYZ>(const ODScenePointCloud<pcl::PointXYZ> &, const std::string &, bool);
	template void ODViewer::update<pcl::PointXYZRGB>(const ODScenePointCloud<pcl::PointXYZRGB> &, const std::string &, bool);
	template void ODViewer::update<pcl::PointXYZRGBA>(const ODScenePointCloud<pcl::PointXYZRGBA> &, const std::string &, bool);

}