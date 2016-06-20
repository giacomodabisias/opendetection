/*
 * PnPProblem.h
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <sstream>
#include "od/detectors/local2D/simple_ransac_detection/ODMesh.h"
#include "od/detectors/local2D/simple_ransac_detection/ODModelRegistration.h"
#include "od/common/utils/ODShared_pointers.h"

class Mesh;
class Ray;
class Triangle;

namespace od {
  
  namespace l2d {

    class PnPProblem
    {

    public:
      explicit PnPProblem(double const param[], double const dists[]);  // custom constructor
      PnPProblem(const cv::Mat & intrin = cv::Mat::eye(3, 4, CV_64F), const cv::Mat & dist = cv::Mat::zeros(1, 5, CV_64F));
      virtual ~PnPProblem();

      cv::Point3f CROSS(const cv::Point3f & v1, const cv::Point3f & v2);
      double DOT(const cv::Point3f & v1, const cv::Point3f & v2);
      cv::Point3f SUB(const cv::Point3f & v1, const cv::Point3f & v2);
      cv::Point3f getNearest3DPoint(std::vector<cv::Point3f> & points_list, const cv::Point3f & origin);
        
      bool backproject2DPoint(const Mesh * mesh, const cv::Point2f & point2d, cv::Point3f & point3d);
      bool intersectMollerTrumbore(Ray & ray, Triangle & Triangle, double & out);
      std::vector<cv::Point2f> verifyPoints(shared_ptr<Mesh> mesh);
      cv::Point2f backproject3DPoint(const cv::Point3f & point3d);
      bool estimatePose(const std::vector<cv::Point3f> & list_points3d, const std::vector<cv::Point2f> & list_points2d, int flags);
      void estimatePoseRANSAC(const std::vector<cv::Point3f> & list_points3d, const std::vector<cv::Point2f> & list_points2d,
                               int flags, const cv::Mat & inliers, int iterations_count, float reprojection_error, double confidence);

      cv::Mat getAMatrix() const;
      cv::Mat getDistCoef() const;
      cv::Mat getRMatrix() const;
      cv::Mat getRVect() const;
      cv::Mat getTMatrix() const;
      cv::Mat getPMatrix() const;

      void setPMatrix(const cv::Mat & r_matrix, const cv::Mat & t_matrix);
      void clearExtrinsics();

    private:

      /** The calibration matrix */
      cv::Mat a_matrix_;
      //dist mat
      cv::Mat dist_;
      /** The computed rotation matrix */
      cv::Mat r_matrix_;
      cv::Mat r_vect_;
      /** The computed translation matrix */
      cv::Mat t_matrix_;
      /** The computed projection matrix */
      cv::Mat p_matrix_;
    };

  }

}
