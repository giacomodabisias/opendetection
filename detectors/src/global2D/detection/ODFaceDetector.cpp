#include "od/detectors/global2D/detection/ODFaceDetector.h"

namespace od
{
	namespace g2d
	{

		ODFaceDetector2D::ODFaceDetector2D(ODFaceDetector2D::FaceRecogType type, const std::string & path) : 
		                  type_(type), scale_(1.1), mn_(2) 
		{

			std::string face_cascade = path.empty() ? "haarcascade_frontalface_alt.xml" : path;
			if(type_ == GPU){
				face_cascade_gpu_ = cv::cuda::CascadeClassifier::create(face_cascade);
				face_cascade_gpu_->setFindLargestObject(false);

				
			}else{
				face_cascade_ = make_shared<cv::CascadeClassifier>(face_cascade);
			}
		}

		shared_ptr<ODDetections2D> ODFaceDetector2D::detectOmni(shared_ptr<ODSceneImage> scene) 
		{

			cv::Mat input = scene->getCVImage();
			faces.clear();

			if(type_ == GPU){
				
				color_gpu_.upload(input);
				cv::cuda::cvtColor(color_gpu_, gray_gpu_, CV_BGR2GRAY);

				face_cascade_gpu_->setScaleFactor(scale_);
				face_cascade_gpu_->detectMultiScale(gray_gpu_, faces_buf_gpu_);
				face_cascade_gpu_->convert(faces_buf_gpu_, faces);

			}else{

				cv::Mat frame_gray;

				cv::cvtColor(input, frame_gray, CV_BGR2GRAY);
				cv::equalizeHist(frame_gray, frame_gray);

				face_cascade_->detectMultiScale(frame_gray, faces, scale_, mn_, 0|CV_HAAR_SCALE_IMAGE, min_size_, max_size_);

			}

			shared_ptr<ODDetections2D> detections = make_shared<ODDetections2D>();

			for(auto & face : faces){
				shared_ptr<ODDetection2D> detection = make_shared<ODDetection2D>(ODDetection::OD_DETECTION, std::string("face"));
				detection->bounding_box_2d_ = face;
				detections->push_back(detection);
			}

			return detections;
		}

		void ODFaceDetector2D::setScale(const float scale)
		{
			scale_ = scale;
		}

		float ODFaceDetector2D::getScale() const
		{
			return scale_;
		}

		void ODFaceDetector2D::setMinNeighbors(const unsigned int mn)
		{
			mn_ = mn;
		}

		unsigned int ODFaceDetector2D::getMinNeighbors() const
		{
			return mn_;
		}

		void ODFaceDetector2D::setMinSize(const cv::Size & size)
		{
			min_size_ = size;
		}

		cv::Size ODFaceDetector2D::getMinSize() const
		{
			return min_size_;
		}

		void ODFaceDetector2D::setMaxSize(const cv::Size & size)
		{
			max_size_ = size;
		}

		cv::Size ODFaceDetector2D::getMaxSize() const
		{
			return max_size_;
		}


		shared_ptr<ODDetections> ODFaceDetector2D::detect(shared_ptr<ODSceneImage> scene)
		{
			std::cout << "Not implemented. Use detectOmi(shared_ptr<ODSceneImage>) instead." << std::endl;
			return nullptr;
		}
	}
}