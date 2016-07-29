/*
 * ModelRegistration.cpp
 *
 *  Created on: Apr 18, 2014
 *      Author: edgar
 */

#include "od/detectors/local2D/simple_ransac_detection/ModelRegistration.h"

namespace od {
  
  namespace l2d {

    void ModelRegistration::setNumMax(int n) 
    {
      max_registrations_ = n; 
    }

    std::vector<cv::Point2f> ModelRegistration::getPoints2d() const 
    {
      return list_points2d_; 
    }

    std::vector<cv::Point3f> ModelRegistration::getPoints3d() const 
    {
      return list_points3d_; 
    }

    int ModelRegistration::getNumMax() const 
    {
      return max_registrations_;
    }

    int ModelRegistration::getNumRegist() const 
    {
      return n_registrations_; 
    }

    bool ModelRegistration::isRegistrable() const 
    {
      return n_registrations_ < max_registrations_; 
    }

    ModelRegistration::ModelRegistration()
    {
      n_registrations_ = 0;
      max_registrations_ = 0;
    }

    ModelRegistration::~ModelRegistration()
    {
      // TO Auto-generated destructor stub
    }

    void ModelRegistration::registerPoint(const cv::Point2f & point2d, const cv::Point3f & point3d)
     {
       // add correspondence at the end of the vector
        list_points2d_.push_back(point2d);
        list_points3d_.push_back(point3d);
        n_registrations_++;
     }

    void ModelRegistration::reset()
    {
      n_registrations_ = 0;
      max_registrations_ = 0;
      list_points2d_.clear();
      list_points3d_.clear();
    }

  }
  
}
