#include "od/common/pipeline/ODDetection.h"

namespace od
{

	ODDetection::ODDetection(const DetectionType & type_, const std::string & id_, double confidence_) : 
	            type_(type_), id_(id_), confidence_(confidence_) {}
	
	void ODDetection::printSelf()
	{
	  std::cout << "--Detection-- \nType: " << enumToString(type_) << std::endl;
	  std::cout << "ID: " << id_ << std::endl;
	}

	const ODDetection::DetectionType & ODDetection::getType() const
	{
	  return type_;
	}

	void ODDetection::setType(const DetectionType & type)
	{
	  type_ = type;
	}

	const std::string & ODDetection::getId() const
	{
	  return id_;
	}

	void ODDetection::setId(const std::string & id)
	{
	  id_ = id_;
	}


	double ODDetection::getConfidence() const
	{
	  return confidence_;
	}


	void ODDetection::setConfidence(double confidence)
	{
	  confidence_ = confidence;
	}




	ODDetection2D::ODDetection2D(const DetectionType & type_, const std::string & id_ , double confidence_ ) : 
	              ODDetection(type_, id_, confidence_)
	{
	  location_2d_ = Eigen::Vector3d::UnitZ();
	}

	const Eigen::Vector3d & ODDetection2D::getLocation() const
	{
	  return location_2d_;
	}

	void ODDetection2D::setLocation(const Eigen::Vector3d & location)
	{
	  location_2d_ = location;
	}

	const cv::Rect & ODDetection2D::getBoundingBox() const
	{
	  return bounding_box_2d_;
	}

	void ODDetection2D::setBoundingBox(const cv::Rect & bounding_box)
	{
	  bounding_box_2d_ = bounding_box;
	}

	const cv::Mat & ODDetection2D::getMetainfoImage() const
	{
	  return metainfo_image_;
	}

	void ODDetection2D::setMetainfoImage(const cv::Mat & metainfo_image)
	{
	  metainfo_image_ = metainfo_image;
	}



	ODDetection3D::ODDetection3D(const DetectionType & type_, const std::string & id_, double confidence_) : 
	              ODDetection(type_, id_, confidence_)
	{
	  location_3d_ = Eigen::Vector4d::UnitW();
	  orientation_.setIdentity();
	  scale_ = 1;
	}

	const Eigen::Vector4d & ODDetection3D::getLocation() const
	{
	  return location_3d_;
	}

	void ODDetection3D::setLocation(const Eigen::Vector4d & location)
	{
	  location_3d_ = location;
	}
	void ODDetection3D::setLocation(const cv::Mat & location)
	{
	  cv::cv2eigen(location, location_3d_);
	}

	const Eigen::Matrix3Xd & ODDetection3D::getPose() const
	{
	  return orientation_;
	}

	void ODDetection3D::setPose(const Eigen::Matrix3Xd & pose)
	{
	  orientation_ = pose;
	}
	void ODDetection3D::setPose(const cv::Mat & pose_cv)
	{
	  orientation_ = Eigen::Map<Eigen::Matrix3d>(pose_cv.clone().ptr<double>());
	}

	double ODDetection3D::getScale() const
	{
	  return scale_;
	}

	void ODDetection3D::setScale(double scale)
	{
	  ODDetection3D::scale_ = scale;
	}

	const cv::Mat & ODDetection3D::getMetainfoImage() const
	{
	  return metainfo_image_;
	}

	void ODDetection3D::setMetainfoImage(const cv::Mat & metainfo_image)
	{
	  metainfo_image_ = metainfo_image;
	}

	const pcl::PointCloud<pcl::PointXYZ>::Ptr & ODDetection3D::getMetainfoCluster() const
	{
	  return metainfo_cluster_;
	}

	void ODDetection3D::setMetainfoCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr & metainfo_cluster)
	{
	  metainfo_cluster_ = metainfo_cluster;
	}

	void ODDetection3D::printSelf()
	{
	  ODDetection::printSelf();
	  std::cout << "Location: " << location_3d_ << std::endl;
	  std::cout << "Pose: " << orientation_ << std::endl;
	  std::cout << "Scale: " << scale_ << std::endl;
	}


	ODDetections::ODDetections(unsigned int n): detections_(n) {}

 	ODDetections::~ODDetections()
	{
	  detections_.resize(0);
	}

	int ODDetections::size() const 
	{
		return detections_.size(); 
	}

	void ODDetections::push_back(shared_ptr<ODDetection> detection)
	{
		detections_.push_back(detection);
	}

	void ODDetections::append(shared_ptr<ODDetections> detections)
	{
		detections_.insert(detections_.end(), detections->detections_.begin(), detections->detections_.end());
	}

	shared_ptr<ODDetection> ODDetections::operator[](unsigned int i)
	{
		return detections_[i]; 
	}
	
	shared_ptr<ODDetection> ODDetections::at(unsigned int i)
	{
		return detections_[i]; 
	}

	const cv::Mat & ODDetections::getMetainfoImage() const
	{
	  	return metainfo_image_;
	}

	void ODDetections::setMetainfoImage(const cv::Mat & metainfo_image)
	{
	 	metainfo_image_ = metainfo_image.clone();
	}

	const pcl::PointCloud<pcl::PointXYZ>::Ptr & ODDetections::getMetainfoCluster() const
	{
	  	return metainfo_cluster_;
	}

	void ODDetections::setMetainfoCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr & metainfo_cluster)
	{
	  	metainfo_cluster_ = metainfo_cluster;
	}


	ODSceneImage ODDetections2D::renderMetainfo(ODSceneImage & input)
	{

	  //picking up random colors for different detection algorithm, if exist
	  /*std::map<std::string, cv::Scalar> color_map;
	  for(int i = 0; i < detections_.size(); i++)
	  {
	    if(color_map.find(detections_[i]->getId()) == color_map.end())
	     color_map[detections_[i]->getId()] = CV_RGB(rand()%255, rand()%255, rand()%255);
	  }*/

	  cv::Mat image = input.getCVImage().clone();
	  for(size_t i = 0; i < detections_.size(); ++i)
	  {
	    shared_ptr<ODDetection2D> detection = dynamic_pointer_cast<ODDetection2D>(detections_[i]);
	    if(detection)
	    	cv::rectangle(image, detection->bounding_box_2d_, getHashedColor(detections_[i]->getId(), 100), 2);
	  }
	  return ODSceneImage(image);
	}


	shared_ptr<ODDetection2D> ODDetections2D::operator[](unsigned int i)
	{
	 	return dynamic_pointer_cast<ODDetection2D>(detections_[i]); 
	}

	shared_ptr<ODDetection2D> ODDetections2D::at(unsigned int i)
	{ 
	 	return dynamic_pointer_cast<ODDetection2D>(detections_[i]); 
	}


	shared_ptr<ODDetection3D> ODDetections3D::operator[](unsigned int i)
	{ 
		return dynamic_pointer_cast<ODDetection3D>(detections_[i]); 
	}

	shared_ptr<ODDetection3D> ODDetections3D::at(unsigned int i) 
	{
		return dynamic_pointer_cast<ODDetection3D>(detections_[i]); 
	}


	shared_ptr<ODDetectionComplete> ODDetectionsComplete::operator[](unsigned int i) 
	{ 
		return dynamic_pointer_cast<ODDetectionComplete>(detections_[i]); 
	}

	shared_ptr<ODDetectionComplete> ODDetectionsComplete::at(unsigned int i) 
	{ 
		return dynamic_pointer_cast<ODDetectionComplete>(detections_[i]); 
	}


}