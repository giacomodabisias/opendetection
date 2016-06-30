
#pragma once
#include "opencv2/highgui/highgui.hpp"
#if WITH_GPU
#include "opencv2/cudaobjdetect.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudawarping.hpp"
#endif
#include "od/common/utils/ODShared_pointers.h"
#include "od/common/pipeline/ODDetector.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace od
{
	namespace g2d
	{

		class ODFaceDetector2D: public ODDetector2D {

#ifndef DOXYGEN_SHOULD_SKIP_THIS

			OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(FaceRecogType, (GPU)(CPU))
#endif

		public:

			ODFaceDetector2D(ODFaceDetector2D::FaceRecogType type, const std::string & path = std::string());

		    shared_ptr<ODDetections> detect(shared_ptr<ODSceneImage> scene);
		    shared_ptr<ODDetections2D> detectOmni(shared_ptr<ODSceneImage> scene);


		    void setScale(const float scale);
		    float getScale() const;

		    void setMinNeighbors(const unsigned int mn);
		    unsigned int getMinNeighbors() const;

		    void setMinSize(const cv::Size & size);
		    cv::Size getMinSize() const;

		    void setMaxSize(const cv::Size & size);
		    cv::Size getMaxSize() const;

		private:
			FaceRecogType type_;
			std::vector<cv::Rect> faces;
			cv::Ptr<cv::cuda::CascadeClassifier> face_cascade_gpu_;
			float scale_;
			unsigned int mn_;
			cv::Size min_size_, max_size_;

#ifdef WITH_GPU
			shared_ptr<cv::CascadeClassifier> face_cascade_;
			cv::cuda::GpuMat gray_gpu_, faces_buf_gpu_, color_gpu_;
#endif

		};
	}
}