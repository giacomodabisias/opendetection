/*
Copyright (c) 2015, Kripasindhu Sarkar
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by sarkar on 12.06.15.
//
#pragma once
#include "od/common/utils/Utils.h"
#include "od/common/pipeline/Scene.h"
#include "od/common/utils/Shared_pointers.h"

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace od
{

  /** \brief The base class of all the detection.
   *
   * This is the base class of all the detection classes containing the detection information. All the Detectors return a collection of this class (in the form of Detections).
   * Supports three modes: recognition (with type RECOGNITION) and classification (with type CLASSIFICATION) and detection (with type DETECTION). Along with the type, Detector sets an ID to identify what class or what instance of recognition is detected/recognied.
   *
   * \author Kripasindhu Sarkar
   *
   */
  namespace detection {
    _DEFINE_ENUM_WITH_STRING_CONVERSIONS(DetectionType, (RECOGNITION)(CLASSIFICATION)(DETECTION)(DETECTION_NULL))
  }


  class Detection
  {
  public:

    

    virtual ~Detection(){}

    /** \brief Constructor of the Detection class.
        \param type The detection type.
        \param id The detection identifier.
        \param confidence The detection confidence.
      */
    Detection(const detection::DetectionType & type = detection::DETECTION_NULL, const std::string & id = std::string(""), double confidence = 1.0);

    /** \brief Prints type and id of the detection. 
      */
    void printSelf();

    /** \brief Get the type of the detection. This can be _RECOGNITION, _CLASSIFICATION, _DETECTION, _DETECTION_NULL.
        \return The type of the detection.
      */
    const detection::DetectionType & getType() const;

    /** \brief Set the type of the detection. This can be _RECOGNITION, _CLASSIFICATION, _DETECTION, _DETECTION_NULL.
        \param type The type of the detection.
      */
    void setType(const detection::DetectionType & type);

    /** \brief Get the id of the detection.
        \return The id of the detection.
      */
    const std::string & getId() const;

    /** \brief Set the id of the detection.
        \param id The id of the detection.
      */
    void setId(const std::string & id);

    /** \brief Get the confidence of the detection. Detector can use this to provide confidence amnong several detections.
        \return The confidence level of the detection.
      */
    double getConfidence() const;

    /** \brief Set the confidence of the detection. Detector can use this to provide confidence amnong several detections.
        \param confidence The confidence level of the detection.
      */
    void setConfidence(double confidence);

  private:

    detection::DetectionType type_;
    std::string id_;
    double confidence_;

  };

  /** \brief Detection for 2D with 2D location information
   *
   * \author Kripasindhu Sarkar
   *
   */
  class Detection2D : public virtual Detection
  {
  public:

    virtual ~Detection2D(){}

    /** \brief Constructor of the Detection2D class.
        \param type The detection type.
        \param id The detection identifier.
        \param confidence The detection confidence.
      */
    Detection2D(const detection::DetectionType & type = detection::DETECTION_NULL, const std::string & id = std::string(""), double confidence = 1.0);

    /** \brief Get the 2D location of the detection.
        \return The 2D vector of the detection position.
      */
    const Eigen::Vector3d & getLocation() const;

    /** \brief Set the 2D location of the detection.
        \param location The 2D location of the detection.
      */
    void setLocation(const Eigen::Vector3d & location);

    /** \brief Get the bounding box of the detection.
        \return The bounding box if the detection in the frame.
      */
    const cv::Rect & getBoundingBox() const;

    /** \brief Set the bounding box of the detection.
        \param bounding_box The bounding box of the detection.
      */
    void setBoundingBox(const cv::Rect & bounding_box);

    /** \brief Get the image template concerning the detection.
        \return Image template concerning the detection.
      */
    const cv::Mat & getMetainfoImage() const;

    /** \brief Set the image template concerning the detection.
        \param metainfo_image The image template concerning the detection.
      */
    void setMetainfoImage(const cv::Mat & metainfo_image);

    Eigen::Vector3d location_2d_;
    cv::Rect bounding_box_2d_;
    cv::Mat metainfo_image_;

  };

  /** \brief Detection in 3D with 3D location information.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class Detection3D : public virtual Detection
  {
  public:

    virtual ~Detection3D(){}

    /** \brief Constructor of the Detection3D class.
        \param type The detection type.
        \param id The detection identifier.
        \param confidence The detection confidence.
      */
    Detection3D(const detection::DetectionType & type = detection::DETECTION_NULL, const std::string & id = std::string(""), double confidence = 1.0);


    /** \brief Get the 3D location of the detection.
        \return The 3D vector of the detection position.
      */
    const Eigen::Vector4d & getLocation() const;

    /** \brief Set the 3D location of the detection.
        \param location 3D location of the detection.
      */
    void setLocation(const Eigen::Vector4d & location);

    /** \brief Set the 3D location of the detection.
        \param location The 3D location of the detection.
      */
    void setLocation(const cv::Mat & location);

    /** \brief Returns the pose of the detection.
        \return The detection orientation.
      */
    const Eigen::Matrix3Xd & getPose() const;

    /** \brief Set the 3D pose of the detection.
        \param pose The 3D pose of the detection.
      */
    void setPose(const Eigen::Matrix3Xd & pose);

    /** \brief Set the 3D location of the detection.
        \param pose_cv The 3D pose of the detection.
      */
    void setPose(const cv::Mat & pose_cv);

    /** \brief Get the scale of the detection.
        \return Scale of the detection.
      */
    double getScale() const;

    /** \brief Set the scale of the detection.
        \param scale scale of the detection.
      */
    void setScale(double scale);

    const cv::Mat & getMetainfoImage() const;
    void setMetainfoImage(const cv::Mat & metainfo_image);

    const shared_ptr<pcl::PointCloud<pcl::PointXYZ> > & getMetainfoCluster() const;
    void setMetainfoCluster(const shared_ptr<pcl::PointCloud<pcl::PointXYZ> > & metainfo_cluster);

    void printSelf();

    Eigen::Vector4d location_3d_;
    Eigen::Matrix3Xd orientation_;
    double scale_;
    cv::Mat metainfo_image_;
    shared_ptr<pcl::PointCloud<pcl::PointXYZ>> metainfo_cluster_;

  };

  /** \brief Detection in 2D with complete information.
  *
  * \author Kripasindhu Sarkar
  *
  */
  class DetectionComplete: public Detection2D, public Detection3D
  {
  };

  /** \brief The container class for Detections
  *
  * \author Kripasindhu Sarkar
  *
  */
  class Detections
  {
  public:

    Detections(unsigned int n = 0);

    virtual ~Detections();

    unsigned int size() const;

    void push_back(shared_ptr<Detection> detection);

    void append(shared_ptr<Detections> detections);

    shared_ptr<Detection> operator[](unsigned int i);
    shared_ptr<Detection> at(unsigned int i);

    const cv::Mat & getMetainfoImage() const;
    void setMetainfoImage(const cv::Mat & metainfo_image_);

    const shared_ptr<pcl::PointCloud<pcl::PointXYZ> > & getMetainfoCluster() const;
    void setMetainfoCluster(const shared_ptr<pcl::PointCloud<pcl::PointXYZ> > & metainfo_cluster);

  protected:

    std::vector<shared_ptr<Detection>> detections_;
    cv::Mat metainfo_image_;
    shared_ptr<pcl::PointCloud<pcl::PointXYZ> > metainfo_cluster_;

  };




/** \brief The container class for Detection2D returned by Detector2D
  *
  * \author Kripasindhu Sarkar
  *
  */
  class Detections2D: public Detections
  {
  public:

    /** \brief Draws rectangles over the input image using the bounding box information present in all the 2D detections. This is a quick function to render and verify the detections made.
      */
    SceneImage renderMetainfo(SceneImage & input);
    SceneImage renderMetainfo(shared_ptr<SceneImage> input);

    shared_ptr<Detection2D> operator[](unsigned int i);
    shared_ptr<Detection2D> at(unsigned int i);

  };

  /** \brief The container class for Detection3D returned by Detector3D
  *
  * \author Kripasindhu Sarkar
  *
  */
  class Detections3D: public Detections
  {
  public:

    /*SceneImage renderMetainfo(SceneImage input)
    {
      //picking up random colors for different detection algorithm, if exist
      std::map<std::string, cv::Scalar> color_map;
      for(int i = 0; i < detections_.size(); i++)
      {
        if(color_map.find(detections_[i]->getId()) == color_map.end())
          color_map[detections_[i]->getId()] = CV_RGB(rand()%255, rand()%255, rand()%255);
      }

      cv::Mat image = input.getCVImage().clone();
      for(int i = 0; i < detections_.size(); i++)
      {
        Detections3D * detection = dynamic_cast<Detections3D *>(detections_[i]);
        cv::rectangle(image, detection->bounding_box_2d_, color_map[detections_[i]->getId()], 2);
      }
      return SceneImage(image);
    }*/


    shared_ptr<Detection3D> operator[](unsigned int i);
    shared_ptr<Detection3D> at(unsigned int i);
  };

  /** \brief The container class for DetectionComplete returned by Detector2DComplete
 *
 * \author Kripasindhu Sarkar
 *
 */
  class DetectionsComplete: public Detections
  {
  public:

    shared_ptr<DetectionComplete> operator[](unsigned int i);
    shared_ptr<DetectionComplete> at(unsigned int i);
  };

}
