#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string>

#include <vtkRenderWindow.h>

#include "od/common/utils/ODShared_pointers.h"
#include "od/common/pipeline/ODScene.h"


namespace od {

	/** \brief The viewer class.
	 *
	 * This class is used to visualize all types handled by the OpenDetection Library. For now it is possible to display cv::Mat and pcl::PointCloud<PointT> >.
	 *
	 * \author Giacomo Dabisias
	 *
	 */
	class ODViewer {

		enum odViewType {
			UNDEFINED,
			POINTCLOUD,
			CVMAT
		};

	public:

		ODViewer();

		template<typename PointT>
		void render(shared_ptr<pcl::PointCloud<PointT> > to_display, const std::string & cloud_name, bool colored = true);

		template<typename PointT>
		void render(shared_ptr<ODScenePointCloud<PointT> > to_display, const std::string & cloud_name, bool colored = true);

		template<typename PointT>
		void render(const ODScenePointCloud<PointT> & to_display, const std::string & cloud_name, bool colored = true);

		template<typename PointT>
		void render(shared_ptr<pcl::PointCloud<PointT> > to_display, 
				    pcl::visualization::PointCloudColorHandlerRandom<PointT> & random_handler, const std::string & cloud_name);

		void render(const cv::Mat & to_display, const std::string & window_name);
		void render(shared_ptr<ODSceneImage> to_display, const std::string & window_name);
		void render(const ODSceneImage & to_display, const std::string & window_name);

		void initCVWindow(const std::string & window_name);
		void initPCLWindow(const std::string & window_name);

		template<typename PointT>
		void update(shared_ptr<pcl::PointCloud<PointT> > to_display, const std::string & cloud_name, bool colored = true);

		template<typename PointT>
		void update(shared_ptr<ODScenePointCloud<PointT> > to_display, const std::string & cloud_name, bool colored = true);

		template<typename PointT>
		void update(const ODScenePointCloud<PointT> & to_display, const std::string & cloud_name, bool colored = true);

		void update(const cv::Mat & to_display, const std::string & window_name);
		void update(const ODSceneImage & to_display, const std::string & window_name);
		void update(shared_ptr<ODSceneImage> to_display, const std::string & window_name);

		void setBackGround(const cv::Scalar & color);
		void setBackGround(unsigned int r, unsigned int g, unsigned int b);

		void registerCallback(void(*callback)(const pcl::visualization::KeyboardEvent &, void *), void * data = nullptr);
		void registerCallback(const std::string & window_name, CvMouseCallback on_mouse, void * data = nullptr);

		void spin();
		bool toStop();

		void addText(const std::string & text, pcl::PointXYZ pos, double textScale, cv::Scalar color);
		void addText(const std::string & text, cv::Point3f pos, double textScale, cv::Scalar color);

		void removeText(const std::string & text);

		void remove(const std::string & name);
		void removeAll();

		void removeShape(const std::string & text);
		void removeAllShapes();

		unsigned int wait(unsigned int time) const;

		shared_ptr<pcl::visualization::PCLVisualizer> getViewer();

	private:

		odViewType status_;
		shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
		std::string mat_window_name_, pcl_window_name_;
		std::vector<std::string> clouds_;
		shared_ptr<cv::Mat> mat_;
		
	};

	extern template void ODViewer::render<pcl::PointXYZ>(shared_ptr<pcl::PointCloud<pcl::PointXYZ> >, const std::string &, bool);
	extern template void ODViewer::render<pcl::PointXYZRGB>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >, const std::string &, bool);
	extern template void ODViewer::render<pcl::PointXYZRGBA>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >, const std::string &, bool);

	extern template void ODViewer::render<pcl::PointXYZ>(shared_ptr<pcl::PointCloud<pcl::PointXYZ> > to_display, 
				    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> & random_handler, const std::string & cloud_name);
	extern template void ODViewer::render<pcl::PointXYZRGB>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > to_display, 
				    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> & random_handler, const std::string & cloud_name);
	extern template void ODViewer::render<pcl::PointXYZRGBA>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > to_display, 
				    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGBA> & random_handler, const std::string & cloud_name);

	extern template void ODViewer::render<pcl::PointXYZ>(shared_ptr<ODScenePointCloud<pcl::PointXYZ> >, const std::string &, bool);
	extern template void ODViewer::render<pcl::PointXYZRGB>(shared_ptr<ODScenePointCloud<pcl::PointXYZRGB> >, const std::string &, bool);
	extern template void ODViewer::render<pcl::PointXYZRGBA>(shared_ptr<ODScenePointCloud<pcl::PointXYZRGBA> >, const std::string &, bool);

	extern template void ODViewer::render<pcl::PointXYZ>(const ODScenePointCloud<pcl::PointXYZ> &, const std::string &, bool);
	extern template void ODViewer::render<pcl::PointXYZRGB>(const ODScenePointCloud<pcl::PointXYZRGB> &, const std::string &, bool);
	extern template void ODViewer::render<pcl::PointXYZRGBA>(const ODScenePointCloud<pcl::PointXYZRGBA> &, const std::string &, bool);

	extern template void ODViewer::update<pcl::PointXYZ>(shared_ptr<pcl::PointCloud<pcl::PointXYZ> >, const std::string &, bool);
	extern template void ODViewer::update<pcl::PointXYZRGB>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >, const std::string &, bool);
	extern template void ODViewer::update<pcl::PointXYZRGBA>(shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >, const std::string &, bool);

	extern template void ODViewer::update<pcl::PointXYZ>(shared_ptr<ODScenePointCloud<pcl::PointXYZ> >, const std::string &, bool);
	extern template void ODViewer::update<pcl::PointXYZRGB>(shared_ptr<ODScenePointCloud<pcl::PointXYZRGB> >, const std::string &, bool);
	extern template void ODViewer::update<pcl::PointXYZRGBA>(shared_ptr<ODScenePointCloud<pcl::PointXYZRGBA> >, const std::string &, bool);

	extern template void ODViewer::update<pcl::PointXYZ>(const ODScenePointCloud<pcl::PointXYZ> &, const std::string &, bool);
	extern template void ODViewer::update<pcl::PointXYZRGB>(const ODScenePointCloud<pcl::PointXYZRGB> &, const std::string &, bool);
	extern template void ODViewer::update<pcl::PointXYZRGBA>(const ODScenePointCloud<pcl::PointXYZRGBA> &, const std::string &, bool);


}

#include "od/common/utils/ODViewer.hpp"
