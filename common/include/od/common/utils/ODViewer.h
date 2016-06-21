#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string>

#include "od/common/utils/ODShared_pointers.h"


namespace od {

		enum odViewType {
			UNDEFINED,
			POINTCLOUD,
			CVMAT
		};

		/** \brief The viewer class.
		 *
		 * This class is used to visualize all types handled by the OpenDetection Library. For now it is possible to display cv::Mat and pcl::PointCloud<PointT> >.
		 *
		 * \author Giacomo Dabisias
		 *
		 */
		class ODViewer {

		public:

			ODViewer();

			template<typename PointT>
			void render(shared_ptr<pcl::PointCloud<PointT>> to_display, const std::string & cloud_name);

			void render(const cv::Mat & to_display, const std::string & window_name);

			template<typename PointT>
			void render(pcl::PointCloud<PointT> & to_display, const std::string & cloud_name);

			template<typename PointT>
			void update(shared_ptr<pcl::PointCloud<PointT>> to_display, const std::string & cloud_name);

			void update(const cv::Mat & to_display, const std::string & window_name);

			template<typename PointT>
			void update(pcl::PointCloud<PointT> & to_display, const std::string & cloud_name);

			void setColorHandler(const std::string & cloud_name);
			void setBackGround(const cv::Scalar & color);
			void setBackGround(unsigned int r, unsigned int g, unsigned int b);

			void registerCallback(void(*callback)(const pcl::visualization::KeyboardEvent &, void *), void * data = nullptr);
			void registerCallback(const std::string & window_name, CvMouseCallback on_mouse, void * data = nullptr);

		private:

			odViewType status_;
			shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

		};

}