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

#include "od/detectors/local2D/detection/ODCADRecognizer2DLocal.h"

namespace od
{
  namespace l2d
  {

    ODCADRecognizer2DLocal::ODCADRecognizer2DLocal(const std::string & trained_data_location_): 
                                                   ODImageLocalMatchingDetector(trained_data_location_)
    {
      meta_info_ = true;

      camera_intrinsic_file_ = "Data/out_camera_data_lion_old.yml";         // mesh

      red_ = cv::Scalar(0, 0, 255);
      green_ = cv::Scalar(0, 255, 0);
      blue_ = cv::Scalar(255, 0, 0);
      yellow_ = cv::Scalar(0, 255, 255);

      num_key_points_ = 2000;      // number of detected keypoints
      ratio_test_ = 0.70f;          // ratio test
      fast_match_ = true;       // fastRobustMatch() or robustMatch()
      use_gpu_ = false;
      use_gpu_match_ = false;

      iterations_count_ = 500;      // number of Ransac iterations.
      reprojection_error_ = 2.0;  // maximum allowed distance to consider it an inlier.
      confidence_ = 0.95;        // ransac successful confidence.

      min_inliers_ = 30;    // Kalman threshold updating

      pnp_method_ = cv::SOLVEPNP_EPNP;
      f_type_default_ = "SIFT";
      feature_detector_ = make_shared<ODFeatureDetector2D>(f_type_default_, use_gpu_);
    }

    const std::string & ODCADRecognizer2DLocal::getCameraIntrinsicFile() const
    {
      return camera_intrinsic_file_;
    }

    void ODCADRecognizer2DLocal::setCameraIntrinsicFile(const std::string & camera_intrinsic_file)
    {
      camera_intrinsic_file_ = camera_intrinsic_file;
    }

    int ODCADRecognizer2DLocal::getNumKeyPoints() const
    {
      return num_key_points_;
    }

    void ODCADRecognizer2DLocal::setNumKeyPoints(int num_key_points)
    {
      num_key_points_ = num_key_points;
    }

    float ODCADRecognizer2DLocal::getRatioTest() const
    {
      return ratio_test_;
    }

    void ODCADRecognizer2DLocal::setRatioTest(float ratio_test)
    {
      ratio_test_ = ratio_test;
    }

    bool ODCADRecognizer2DLocal::isFastMatch() const
    {
      return fast_match_;
    }

    void ODCADRecognizer2DLocal::setFastMatch(bool fast_match)
    {
      fast_match_ = fast_match;
    }

    bool ODCADRecognizer2DLocal::isUseGpu() const
    {
      return use_gpu_;
    }

    void ODCADRecognizer2DLocal::setUseGpu(bool use_gpu)
    {
      use_gpu_ = use_gpu;
    }

    bool ODCADRecognizer2DLocal::isUseGpuMatch() const
    {
      return use_gpu_match_;
    }

    void ODCADRecognizer2DLocal::setUseGpuMatch(bool use_gpu_match)
    {
      use_gpu_match_ = use_gpu_match;
    }

    bool ODCADRecognizer2DLocal::isMetainfo() const
    {
      return meta_info_;
    }

    void ODCADRecognizer2DLocal::setMetainfo(bool meta_info)
    {
      meta_info_ = meta_info;
    }

    int ODCADRecognizer2DLocal::getIterationsCount() const
    {
      return iterations_count_;
    }

    void ODCADRecognizer2DLocal::setIterationsCount(int iterations_count)
    {
      iterations_count_ = iterations_count;
    }

    float ODCADRecognizer2DLocal::getReprojectionError() const
    {
      return reprojection_error_;
    }

    void ODCADRecognizer2DLocal::setReprojectionError(float reprojection_error)
    {
      reprojection_error_ = reprojection_error;
    }

    double ODCADRecognizer2DLocal::getConfidence() const
    {
      return confidence_;
    }

    void ODCADRecognizer2DLocal::setConfidence(double confidence)
    {
      confidence_ = confidence;
    }

    int ODCADRecognizer2DLocal::getMinInliers() const
    {
      return min_inliers_;
    }

    void ODCADRecognizer2DLocal::setMinInliers(int min_inliers)
    {
      min_inliers_ = min_inliers;
    }

    int ODCADRecognizer2DLocal::getPnpMethod() const
    {
      return pnp_method_;
    }

    void ODCADRecognizer2DLocal::setPnpMethod(int pnp_method)
    {
      pnp_method_ = pnp_method;
    }

    void ODCADRecognizer2DLocal::parseParameterString(const std::string & parameter_string)
    {
      const std::string keys = "{help h        |      | print this message                   }"
          "{video v       |      | path to recorded video               }"
          "{test_images img       |      | image for detection               }"
          "{cam_id         |      | pass true if you want the input from the camera             }"
          "{camera_intrinsic_file      |      | yml file containing camera parameters             }"
          "{use_gpu        |      | use gpu or cpu                     }"
          "{use_gpu_match        |      | use gpu or cpu for matching                    }"
          "{keypoints k   |2000  | number of keypoints to detect        }"
          "{ratio r       |0.7   | threshold for ratio test             }"
          "{iterations it |500   | RANSAC maximum iterations count      }"
          "{error e       |2.0   | RANSAC reprojection errror           }"
          "{confidence c  |0.95  | RANSAC confidence                    }"
          "{inliers in    |30    | minimum inliers for Kalman update    }"
          "{method  pnp   |0     | PnP method: (0) ITERATIVE - (1) EPNP - (2) P3P - (3) DLS}"
          "{fast f        |false  | use of robust fast match             }"
          "{metainfo meta        |  | provide meta information in Detection             }";

      char **argv;
      int argc;
      fileutils::getArgvArgc(parameter_string, &argv, argc);

      cv::CommandLineParser parser(argc, argv, keys);
      if(parser.has("help"))
      {
        parser.printMessage();

      } else
      {
        camera_intrinsic_file_ =
            parser.get<std::string>("camera_intrinsic_file").size() > 0 ? parser.get<std::string>("camera_intrinsic_file") : camera_intrinsic_file_;

        use_gpu_ = parser.has("use_gpu");
        use_gpu_match_ = parser.has("use_gpu_match");
        num_key_points_ = !parser.has("keypoints") ? parser.get<int>("keypoints") : num_key_points_;
        ratio_test_ = !parser.has("ratio") ? parser.get<float>("ratio") : ratio_test_;
        fast_match_ = !parser.has("fast") ? parser.get<bool>("fast") : fast_match_;
        iterations_count_ = !parser.has("iterations") ? parser.get<int>("iterations") : iterations_count_;
        reprojection_error_ = !parser.has("error") ? parser.get<float>("error") : reprojection_error_;
        confidence_ = !parser.has("confidence") ? parser.get<float>("confidence") : confidence_;
        min_inliers_ = !parser.has("inliers") ? parser.get<int>("inliers") : min_inliers_;
        pnp_method_ = !parser.has("method") ? parser.get<int>("method") : pnp_method_;
        meta_info_ = parser.has("metainfo");
      }

    }

    void ODCADRecognizer2DLocal::init()
    {
      cv::FileStorage fs(camera_intrinsic_file_, cv::FileStorage::READ);
      cv::Mat cam_man, dist_coeff;
      fs["Camera_Matrix"] >> cam_man;
      fs["Distortion_Coefficients"] >> dist_coeff;
      pnp_detection_ = PnPProblem(cam_man, dist_coeff);

      // get all trained models
      fileutils::getFilesInDirectoryRec(getSpecificTrainingDataLocation(), TRAINED_DATA_ID_, model_names_);

      for(size_t i = 0; i < model_names_.size(); ++i)
      {
        Model model;
        model.loadNewXml(model_names_[i]);
        models_.push_back(model);
      }
      if(models_.size() > 0)
        f_type_default_ = models_[0].f_type_;

      feature_detector_ = make_shared<ODFeatureDetector2D>(f_type_default_, use_gpu_);

    }



    shared_ptr<ODDetections> ODCADRecognizer2DLocal::detect(shared_ptr<ODSceneImage> scene)
    {
      shared_ptr<ODDetections3D> detections = detectOmni(scene);
      return dynamic_pointer_cast<ODDetections>(detections);
    }

    bool ODCADRecognizer2DLocal::detectSingleModel(shared_ptr<ODSceneImage> scene, const Model & model, shared_ptr<ODDetection3D> & detection3D, 
                                                   const cv::Mat & frame_vis)
    {

      //reset
      pnp_detection_.clearExtrinsics();

      std::vector<cv::Point3f> list_points3d_model = model.getPoints3d();  // list with model 3D coordinates
      std::vector<cv::KeyPoint> list_keypoints_model = model.getKeypoints();  // list with model 3D coordinates
      cv::Mat descriptors_model = model.getDescriptors();                  // list with descriptors of each 3D coordinate

      RobustMatcher rmatcher(model, use_gpu_, use_gpu_match_); // instantiate RobustMatcher


      // -- Step 1: Robust matching between model descriptors and scene descriptors
      std::vector<cv::DMatch> good_matches;       // to obtain the 3D points of the model
      std::vector<cv::KeyPoint> keypoints_scene = scene->getKeypoints();  // to obtain the 2D points of the scene

      cv::Mat frame = scene->getCVImage();

      //normalize
      cv::Mat scenedes = scene->getDescriptors();
      rmatcher.matchNormalized(scenedes, descriptors_model, good_matches);
      //rmatcher.match(scenedes, descriptors_model, good_matches);

      if(good_matches.size() <= 0) 
        return false;

      std::vector<cv::Point3f> list_points3d_model_match; // container for the model 3D coordinates found in the scene
      std::vector<cv::KeyPoint> list_keypoints_model_match; // container for the model 2D coordinates in the textured image of the corresponding 3D coordinates
      std::vector<cv::Point2f> list_points2d_scene_match; // container for the model 2D coordinates found in the scene

      for(size_t match_index = 0; match_index < good_matches.size(); ++match_index)
      {
        cv::Point3f point3d_model = list_points3d_model[good_matches[match_index].trainIdx];  // 3D point from model
        cv::KeyPoint kp_model = list_keypoints_model[good_matches[match_index].trainIdx];  // 2D point from model
        cv::Point2f point2d_scene = keypoints_scene[good_matches[match_index].queryIdx].pt; // 2D point from the scene

        list_points3d_model_match.push_back(point3d_model);         // add 3D point
        list_keypoints_model_match.push_back(kp_model);
        list_points2d_scene_match.push_back(point2d_scene);         // add 2D point
      }


      cv::Mat inliers_idx;
      std::vector<cv::Point2f> list_points2d_inliers;


      // -- Step 3: Estimate the pose using RANSAC approach
      pnp_detection_.estimatePoseRANSAC(list_points3d_model_match, list_points2d_scene_match, pnp_method_, inliers_idx,
                                       iterations_count_, reprojection_error_, confidence_);

      if(inliers_idx.rows < min_inliers_) 
        return false;

      std::cout << "Recognized: " << model.id_ << std::endl;
      //else everything is fine; report the detection
      if(!detection3D)
        detection3D = make_shared<ODDetection3D>();
      detection3D->setLocation(pnp_detection_.getTMatrix());
      detection3D->setPose(pnp_detection_.getRMatrix());
      detection3D->setType(ODDetection::OD_DETECTION_RECOG);
      detection3D->setId(model.id_);

      return true;
    }

    shared_ptr<ODDetections3D> ODCADRecognizer2DLocal::detectOmni(shared_ptr<ODSceneImage> scene)
    {

      std::vector<cv::KeyPoint> keypoints_scene;
      cv::Mat descriptor_scene;
      feature_detector_->computeKeypointsAndDescriptors(scene->getCVImage(), descriptor_scene, keypoints_scene);
      scene->setDescriptors(descriptor_scene);
      scene->setKeypoints(keypoints_scene);

      shared_ptr<ODDetections3D> detections = make_shared<ODDetections3D>();
      cv::Mat viz = scene->getCVImage().clone();

      for(size_t i = 0; i < models_.size(); ++i)
      {

        shared_ptr<ODDetection3D> detection = make_shared<ODDetection3D>();
        if(detectSingleModel(scene, models_[i], detection, viz))
        {
          detections->push_back(detection);

          if(meta_info_)
            drawModel(viz, &models_[i], pnp_detection_.getRVect(), pnp_detection_.getTMatrix(), pnp_detection_.getAMatrix(), 
                      pnp_detection_.getDistCoef(),  yellow_);
        }
      }
      detections->setMetainfoImage(viz);
      return detections;
    }

  }
}