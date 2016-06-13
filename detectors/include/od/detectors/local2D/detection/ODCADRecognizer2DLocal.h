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
// Created by sarkar on 08.06.15.
//
#pragma once
#include "od/detectors/local2D/ODImageLocalMatching.h"
#include "od/common/pipeline/ODScene.h"
#include "od/detectors/local2D/simple_ransac_detection/Mesh.h"
#include "od/detectors/local2D/simple_ransac_detection/RobustMatcher.h"

#include <iostream>

namespace od
{

  namespace l2d
  {

    /** \brief Simple RANSAC based 3D object recognizer.
     *
     * A recognizer which uses local features like SIFT/SURF to perform object recognition.
     * Given a 'trained model' trained by ODCADRecogTrainerSnapshotBased or trained externally (manually augmenting features in 3D cad models), this class performs a complete detection
     * in an image. It first extracts 2D features from the scene, matches them with all the feature augmented models (the trained data) and in the end solves PnP under RANSAC.
     *
     * \author Kripasindhu Sarkar
     *
     */
    class ODCADRecognizer2DLocal : public ODImageLocalMatchingDetector
    {
    public:

      ODCADRecognizer2DLocal(const std::string & trained_data_location_ = 0);

      const std::string & getCameraIntrinsicFile() const;

      void setCameraIntrinsicFile(const std::string & camera_intrinsic_file);

      int getNumKeyPoints() const;
      void setNumKeyPoints(int num_key_points);

      float getRatioTest() const;
      void setRatioTest(float ratio_test);

      bool isFastMatch() const;
      void setFastMatch(bool fast_match);

      bool isUseGpu() const;
      void setUseGpu(bool use_gpu);

      bool isUseGpuMatch() const;
      void setUseGpuMatch(bool use_gpu_match);

      bool isMetainfo() const;
      void setMetainfo(bool meta_info);

      int getIterationsCount() const;
      void setIterationsCount(int iterations_count);

      float getReprojectionError() const;
      void setReprojectionError(float reprojection_error);

      double getConfidence() const;
      void setConfidence(double confidence);

      int getMinInliers() const;
      void setMinInliers(int min_inliers);

      int getPnpMethod() const;
      void setPnpMethod(int pnp_method);

      void parseParameterString(const std::string & parameter_string);

      void init();

      shared_ptr<ODDetections> detect(shared_ptr<ODSceneImage> scene);
      shared_ptr<ODDetections3D> detectOmni(shared_ptr<ODSceneImage> scene);

    protected:

      bool detectSingleModel(shared_ptr<ODSceneImage> scene, const Model & model, shared_ptr<ODDetection3D> & detection3D, const cv::Mat & frame_viz);

      std::string camera_intrinsic_file_;      

      cv::Scalar red_;
      cv::Scalar green_;
      cv::Scalar blue_;
      cv::Scalar yellow_;

// Robust Matcher parameters
      int num_key_points_;      // number of detected keypoints
      float ratio_test_;          // ratio test
      bool fast_match_;       // fastRobustMatch() or robustMatch()
      bool use_gpu_;
      bool use_gpu_match_;

// RANSAC parameters
      int iterations_count_;      // number of Ransac iterations.
      float reprojection_error_;  // maximum allowed distance to consider it an inlier.
      double confidence_;        // ransac successful confidence.

// Kalman Filter parameters
      int min_inliers_;    // Kalman threshold updating

// PnP parameters
      int pnp_method_;

      //############NON-CONFIG PARAMETERS used for detection###########
      std::vector<std::string> model_names_;
      std::vector<Model> models_;
      PnPProblem pnp_detection_;
      std::string f_type_default_;
      shared_ptr<ODFeatureDetector2D> feature_detector_;

    };
    /** \example objectdetector/od_image_cadrecog_camera.cpp
     * \example objectdetector/od_image_cadrecog_files.cpp
     */
  }
}

