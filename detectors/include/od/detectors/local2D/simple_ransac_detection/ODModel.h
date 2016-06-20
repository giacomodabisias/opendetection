/*
 * Model.h
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */
#pragma once 
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pugixml.hpp>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include "od/detectors/local2D/simple_ransac_detection/ODCsvWriter.h"


namespace od {
  
  namespace l2d {

    class Model
    {
    public:
      Model();
      virtual ~Model();

      std::vector<cv::Point2f> getPoints2dIn() const;
      std::vector<cv::Point2f> getPoints2dOut() const;
      std::vector<cv::Point3f> getPoints3d() const;
      std::vector<cv::KeyPoint> getKeypoints() const;
      cv::Mat getDescriptors() const;
      int getNumDescriptors() const;

      void addCorrespondence(const cv::Point2f &point2d, const cv::Point3f &point3d);
      void addOutlier(const cv::Point2f &point2d);
      void addDescriptor(const cv::Mat &descriptor);
      void addKeypoint(const cv::KeyPoint &kp);

      void save(const std::string & path);
      void load(const std::string & path);
      void loadNewDesc(const std::string & path);
      void loadNewXml(const std::string & path);

      std::string f_type_;
      std::string id_;

    private:

      /** The current number of correspondecnes */
      int n_correspondences_;
      /** The list of 2D points on the model surface */
      std::vector<cv::KeyPoint> list_keypoints_;
      /** The list of 2D points on the model surface */
      std::vector<cv::Point2f> list_points2d_in_;
      /** The list of 2D points outside the model surface */
      std::vector<cv::Point2f> list_points2d_out_;
      /** The list of 3D points on the model surface */
      std::vector<cv::Point3f> list_points3d_in_;
      /** The list of 2D points descriptors */
      cv::Mat descriptors_;

      unsigned int getDescriptorSize();
    };

  }
  
}
