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
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by sarkar on 12.06.15.
//
#pragma once
#include "od/common/utils/ODUtils.h"
#include "od/common/pipeline/ODScene.h"
#include "od/common/utils/ODShared_pointers.h"

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace od
{

  /** \brief The base class of all the detection.
   *
   * This is the base class of all the detection classes containing the detection information. All the ODDetector s return a collection of this class (in the form of ODDetections).
   * Supports two modes: recognition (with type OD_DETECTION_RECOG) and classification (with type OD_DETECTION_CLASS). Along with the type, ODDetector sets an ID to identify what class or what instance of recognition is detected/recognied.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODDetection
  {
  public:


    OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(DetectionType, (OD_DETECTION_RECOG)(OD_DETECTION_CLASS)(OD_DETECTION_NULL))

    virtual ~ODDetection() {}

    ODDetection(const DetectionType & type_ = OD_DETECTION_NULL, const std::string & id_ = "", double confidence_ = 1.0);

    void printSelf();

    const DetectionType & getType() const;
    void setType(const DetectionType & type);

    const std::string & getId() const;
    void setId(const std::string & id);

    /** \brief Get/Set the confidence of the detection. ODDetector can use this to provide confidence amnong several detections.
      */
    double getConfidence() const;

    /** \brief Get/Set the confidence of the detection. ODDetector can use this to provide confidence amnong several detections.
      */
    void setConfidence(double confidence);

  private:

    DetectionType type_;
    std::string id_;
    double confidence_;

  };

  /** \brief Detection for 2D with 2D location information
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ODDetection2D : public virtual ODDetection
  {
  public:

    virtual ~ODDetection2D(){}

    ODDetection2D(const DetectionType & type_ = OD_DETECTION_NULL, const std::string & id_ = "", double confidence_ = 1.0);

    const Eigen::Vector3d & getLocation() const;

    void setLocation(const Eigen::Vector3d & location);

    const cv::Rect & getBoundingBox() const;
    void setBoundingBox(const cv::Rect & bounding_box);

    const cv::Mat & getMetainfoImage() const;
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
  class ODDetection3D : public virtual ODDetection
  {
  public:

    virtual ~ODDetection3D(){}

    ODDetection3D(const DetectionType & type_ = OD_DETECTION_NULL, const std::string & id_ = "", double confidence_ = 1.0);

    const Eigen::Vector4d & getLocation() const;

    void setLocation(const Eigen::Vector4d & location);
    void setLocation(const cv::Mat & location);

    const Eigen::Matrix3Xd & getPose() const;

    void setPose(const Eigen::Matrix3Xd & pose);
    void setPose(const cv::Mat & pose_cv);

    double getScale() const;
    void setScale(double scale);

    const cv::Mat & getMetainfoImage() const;
    void setMetainfoImage(const cv::Mat & metainfo_image);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr & getMetainfoCluster() const;
    void setMetainfoCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr & metainfo_cluster);

    void printSelf();

    Eigen::Vector4d location_3d_;
    Eigen::Matrix3Xd orientation_;
    double scale_;
    cv::Mat metainfo_image_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr metainfo_cluster_;

  };

  /** \brief Detection in 2D with complete information.
  *
  * \author Kripasindhu Sarkar
  *
  */
  class ODDetectionComplete: public ODDetection2D, public ODDetection3D
  {
  };

  /** \brief The container class for ODDetection
  *
  * \author Kripasindhu Sarkar
  *
  */
  class ODDetections
  {
  public:

    ODDetections (unsigned int n = 0);

    virtual ~ODDetections();

    unsigned int size() const;

    void push_back(shared_ptr<ODDetection> detection);

    void append(shared_ptr<ODDetections> detections);

    shared_ptr<ODDetection> operator[](unsigned int i);
    shared_ptr<ODDetection> at(unsigned int i);

    const cv::Mat & getMetainfoImage() const;
    void setMetainfoImage(const cv::Mat & metainfo_image_);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr & getMetainfoCluster() const;
    void setMetainfoCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr & metainfo_cluster);

  protected:

    std::vector<shared_ptr<ODDetection>> detections_;
    cv::Mat metainfo_image_;
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr metainfo_cluster_;

  };




/** \brief The container class for ODDetection2D returned by ODDetector2D
  *
  * \author Kripasindhu Sarkar
  *
  */
  class ODDetections2D: public ODDetections
  {
  public:

    /** \brief Draws rectangles over the input image using the bounding box information present in all the 2D detections. This is a quick function to render and verify the detections made.
      */
    ODSceneImage renderMetainfo(ODSceneImage & input);

    shared_ptr<ODDetection2D> operator[](unsigned int i);
    shared_ptr<ODDetection2D> at(unsigned int i);

  };

  /** \brief The container class for ODDetection3D returned by ODDetector3D
  *
  * \author Kripasindhu Sarkar
  *
  */
  class ODDetections3D: public ODDetections
  {
  public:

    /*ODSceneImage renderMetainfo(ODSceneImage input)
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
        ODDetections3D * detection = dynamic_cast<ODDetections3D *>(detections_[i]);
        cv::rectangle(image, detection->bounding_box_2d_, color_map[detections_[i]->getId()], 2);
      }
      return ODSceneImage(image);
    }*/


    shared_ptr<ODDetection3D> operator[](unsigned int i);
    shared_ptr<ODDetection3D> at(unsigned int i);
  };

  /** \brief The container class for ODDetectionComplete returned by ODDetector2DComplete
 *
 * \author Kripasindhu Sarkar
 *
 */
  class ODDetectionsComplete: public ODDetections
  {
  public:

    shared_ptr<ODDetectionComplete>  operator[](unsigned int i);
    shared_ptr<ODDetectionComplete> at(unsigned int i);
  };

}
