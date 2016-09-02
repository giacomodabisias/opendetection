#include "od/common/pipeline/Detection.h"

namespace od
{

	Detection::Detection(const detection::DetectionType & type, const std::string & id, double confidence) : 
	            type_(type), id_(id), confidence_(confidence) {}
	
	void Detection::printSelf()
	{
	  std::cout << "--Detection-- \nType: " << enumToString(type_) << std::endl;
	  std::cout << "ID: " << id_ << std::endl;
	}

	const detection::DetectionType & Detection::getType() const
	{
	  return type_;
	}

	void Detection::setType(const detection::DetectionType & type)
	{
	  type_ = type;
	}

	const std::string & Detection::getId() const
	{
	  return id_;
	}

	void Detection::setId(const std::string & id)
	{
	  id_ = id_;
	}

	double Detection::getConfidence() const
	{
	  return confidence_;
	}


	void Detection::setConfidence(double confidence)
	{
	  confidence_ = confidence;
	}




	Detection2D::Detection2D(const detection::DetectionType & type, const std::string & id , double confidence) : Detection(type, id, confidence)
	{
	  location_2d_ = Eigen::Vector3d::UnitZ();
	}

	const Eigen::Vector3d & Detection2D::getLocation() const
	{
	  return location_2d_;
	}

	void Detection2D::setLocation(const Eigen::Vector3d & location)
	{
	  location_2d_ = location;
	}

	const cv::Rect & Detection2D::getBoundingBox() const
	{
	  return bounding_box_2d_;
	}

	void Detection2D::setBoundingBox(const cv::Rect & bounding_box)
	{
	  bounding_box_2d_ = bounding_box;
	}

	const cv::Mat & Detection2D::getMetainfoImage() const
	{
	  return metainfo_image_;
	}

	void Detection2D::setMetainfoImage(const cv::Mat & metainfo_image)
	{
	  metainfo_image_ = metainfo_image;
	}



	Detection3D::Detection3D(const detection::DetectionType & type, const std::string & id, double confidence) : Detection(type, id, confidence)
	{
	  location_3d_ = Eigen::Vector4d::UnitW();
	  orientation_.setIdentity();
	  scale_ = 1;
	}

	const Eigen::Vector4d & Detection3D::getLocation() const
	{
	  return location_3d_;
	}

	void Detection3D::setLocation(const Eigen::Vector4d & location)
	{
	  location_3d_ = location;
	}
	void Detection3D::setLocation(const cv::Mat & location)
	{
	  cv::cv2eigen(location, location_3d_);
	}

	const Eigen::Matrix3Xd & Detection3D::getPose() const
	{
	  return orientation_;
	}

	void Detection3D::setPose(const Eigen::Matrix3Xd & pose)
	{
	  orientation_ = pose;
	}
	//TO why clone?
	void Detection3D::setPose(const cv::Mat & pose_cv)
	{
	  orientation_ = Eigen::Map<Eigen::Matrix3d>(pose_cv.clone().ptr<double>());
	}

	double Detection3D::getScale() const
	{
	  return scale_;
	}

	void Detection3D::setScale(double scale)
	{
	  Detection3D::scale_ = scale;
	}

	const cv::Mat & Detection3D::getMetainfoImage() const
	{
	  return metainfo_image_;
	}

	void Detection3D::setMetainfoImage(const cv::Mat & metainfo_image)
	{
	  metainfo_image_ = metainfo_image;
	}

	const pcl::PointCloud<pcl::PointXYZ>::Ptr & Detection3D::getMetainfoCluster() const
	{
	  return metainfo_cluster_;
	}

	void Detection3D::setMetainfoCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr & metainfo_cluster)
	{
	  metainfo_cluster_ = metainfo_cluster;
	}

	void Detection3D::printSelf()
	{
	  Detection::printSelf();
	  std::cout << "Location: " << location_3d_ << std::endl;
	  std::cout << "Pose: " << orientation_ << std::endl;
	  std::cout << "Scale: " << scale_ << std::endl;
	}


	Detections::Detections(unsigned int n): detections_(n) {}

 	Detections::~Detections()
	{
	  detections_.resize(0);
	}

	unsigned int Detections::size() const 
	{
		return detections_.size(); 
	}

	void Detections::push_back(shared_ptr<Detection> detection)
	{
		detections_.push_back(detection);
	}

	void Detections::append(shared_ptr<Detections> detections)
	{
		detections_.insert(detections_.end(), detections->detections_.begin(), detections->detections_.end());
	}

	shared_ptr<Detection> Detections::operator[](unsigned int i)
	{
		return detections_[i]; 
	}
	
	shared_ptr<Detection> Detections::at(unsigned int i)
	{
		return detections_[i]; 
	}

	const cv::Mat & Detections::getMetainfoImage() const
	{
	  	return metainfo_image_;
	}

	void Detections::setMetainfoImage(const cv::Mat & metainfo_image)
	{
	 	metainfo_image_ = metainfo_image.clone();
	}

	const pcl::PointCloud<pcl::PointXYZ>::Ptr & Detections::getMetainfoCluster() const
	{
	  	return metainfo_cluster_;
	}

	void Detections::setMetainfoCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr & metainfo_cluster)
	{
	  	metainfo_cluster_ = metainfo_cluster;
	}


	SceneImage Detections2D::renderMetainfo(SceneImage & input)
	{

	  //picking up random colors for different detection algorithm, if exist
	  /*std::map<std::string, cv::Scalar> color_map;
	  for(int i = 0; i < detections_.size(); i++)
	  {
	    if(color_map.find(detections_[i]->getId()) == color_map.end())
	     color_map[detections_[i]->getId()] = CV_RGB(rand()%255, rand()%255, rand()%255);
	  }*/

	  cv::Mat image = input.getCVImage();
	  for(size_t i = 0; i < detections_.size(); ++i)
	  {
	    shared_ptr<Detection2D> detection = dynamic_pointer_cast<Detection2D>(detections_[i]);
	    if(detection)
	    	cv::rectangle(image, detection->bounding_box_2d_, getHashedColor(detections_[i]->getId(), 100), 2);
	  }
	  return SceneImage(image);
	}

	SceneImage Detections2D::renderMetainfo(shared_ptr<SceneImage> input)
	{

	  //picking up random colors for different detection algorithm, if exist
	  /*std::map<std::string, cv::Scalar> color_map;
	  for(int i = 0; i < detections_.size(); i++)
	  {
	    if(color_map.find(detections_[i]->getId()) == color_map.end())
	     color_map[detections_[i]->getId()] = CV_RGB(rand()%255, rand()%255, rand()%255);
	  }*/

	  cv::Mat image = input->getCVImage().clone();
	  for(size_t i = 0; i < detections_.size(); ++i)
	  {
	    shared_ptr<Detection2D> detection = dynamic_pointer_cast<Detection2D>(detections_[i]);
	    if(detection)
	    	cv::rectangle(image, detection->bounding_box_2d_, getHashedColor(detections_[i]->getId(), 100), 2);
	  }
	  return SceneImage(image);
	}


	shared_ptr<Detection2D> Detections2D::operator[](unsigned int i)
	{
	 	return dynamic_pointer_cast<Detection2D>(detections_[i]); 
	}

	shared_ptr<Detection2D> Detections2D::at(unsigned int i)
	{ 
	 	return dynamic_pointer_cast<Detection2D>(detections_[i]); 
	}


	shared_ptr<Detection3D> Detections3D::operator[](unsigned int i)
	{ 
		return dynamic_pointer_cast<Detection3D>(detections_[i]); 
	}

	shared_ptr<Detection3D> Detections3D::at(unsigned int i) 
	{
		return dynamic_pointer_cast<Detection3D>(detections_[i]); 
	}


	shared_ptr<DetectionComplete> DetectionsComplete::operator[](unsigned int i) 
	{ 
		return dynamic_pointer_cast<DetectionComplete>(detections_[i]); 
	}

	shared_ptr<DetectionComplete> DetectionsComplete::at(unsigned int i) 
	{ 
		return dynamic_pointer_cast<DetectionComplete>(detections_[i]); 
	}


}