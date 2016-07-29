/*
 * ModelRegistration.h
 *
 *  Created on: Apr 18, 2014
 *      Author: edgar
 */
#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>

namespace od {
  
  namespace l2d {

    class ModelRegistration
    {
    public:

      ModelRegistration();
      virtual ~ModelRegistration();

      void setNumMax(int n);

      std::vector<cv::Point2f> getPoints2d() const;
      std::vector<cv::Point3f> getPoints3d() const;
      int getNumMax() const;
      int getNumRegist() const;

      bool isRegistrable() const;
      void registerPoint(const cv::Point2f & point2d, const cv::Point3f & point3d);
      void reset();

    private:
    /** The current number of registered points */
    int n_registrations_;
    /** The total number of points to register */
    int max_registrations_;
    /** The list of 2D points to register the model */
    std::vector<cv::Point2f> list_points2d_;
    /** The list of 3D points to register the model */
    std::vector<cv::Point3f> list_points3d_;
    };

  }

}