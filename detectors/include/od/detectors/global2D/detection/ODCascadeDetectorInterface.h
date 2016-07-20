#pragma once

namespace od
{
	class ODCascadeDetectorInterface
	{
	public:

		virtual shared_ptr<ODDetections2D> detectOmni(shared_ptr<ODSceneImage> scene) = 0;
		virtual shared_ptr<ODDetections> detect(shared_ptr<ODSceneImage> scene) = 0;

		virtual shared_ptr<ODDetections> detectOmni(shared_ptr<ODScene> scene) = 0;

		virtual void init() = 0;

		virtual void setScale(const float scale) = 0;
		virtual float getScale() const = 0;

		virtual void setMinNeighbors(const unsigned int min_neighbors) = 0;
		virtual unsigned int getMinNeighbors() const = 0;

		virtual void setMinSize(const cv::Size & size) = 0;
		virtual cv::Size getMinSize() const = 0;

		virtual void setMaxSize(const cv::Size & size) = 0;
		virtual cv::Size getMaxSize() const = 0;
	  
	};

}