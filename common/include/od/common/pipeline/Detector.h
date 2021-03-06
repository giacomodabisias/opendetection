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
// Created by sarkar on 08.06.15.
//
#pragma once 
#include "od/common/pipeline/Scene.h"
#include "od/common/pipeline/ObjectDetector.h"
#include "od/common/utils/Shared_pointers.h"

namespace od
{
  /** \brief The main detector class; all special Detectors derives from this.
   *
   * Provides interface for two important function detect() and detectOmni(). detectOmni() performs a detection/recognition on the entire scene (unsegmented and unprocessed)
   * and provides information about the detection as well as its exact location. detect() takes an 'object candidate' or a segmented/processed scene as an input and identifies if the entire scene is a detection.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class Detector: public DetectorCommon
  {
  public:

    Detector(const std::string & training_data_location) : DetectorCommon(training_data_location) {}
    Detector() {}

    virtual shared_ptr<Detections> detect(shared_ptr<Scene> scene) = 0;
    virtual shared_ptr<Detections> detectOmni(shared_ptr<Scene> scene) = 0;

    bool meta_info_;

  };

/** \brief The detector of 2D scene.
   *
   * This class takes a 2D scene (SceneImage) as input and performs detection on them. All the 2D detectors should derive from this class and implement the detect and detectOmni functions.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class Detector2D: public Detector
  {
  public:
    Detector2D(const std::string & trained_data_location) : Detector(trained_data_location) {}
    Detector2D() {}

    shared_ptr<Detections> detect(shared_ptr<Scene> scene)
    {
      return detect(dynamic_pointer_cast<SceneImage>(scene));
    }

    /** \brief Function for performing detection on a segmented scene.
     * The purpose of this function is to perform detection on a segmented scene or an 'object candidate'. i.e. the entire scene is considered as an 'object' or an detection. It is possible for a scene to trigger multiple detections.
     * \param[in] scene An instance of 2D scene
     * \return [out] detections A number of detections as an Detections instance.
    */
    virtual shared_ptr<Detections> detect(shared_ptr<SceneImage> scene) = 0;

    /** \brief Function for performing detection on an entire scene.
     *  The purpose of this function is to detect an object in an entire scene. Thus, other than the type of detection we also have information about the location of the detection w.r.t. the scene.
     * \param[in] scene An instance of 2D scene
     * \return [out] detections A number of detections as an Detections2D instance containing information about the detection and its 2D location.
    */
    virtual shared_ptr<Detections2D> detectOmni(shared_ptr<SceneImage> scene) = 0;
  };

  /** \brief The detector of 3D scene.
   *
   * This class takes a 2D scene (SceneImage) as input and performs detection on them. All the 3D detectors should derive from this class and implement the detect and detectOmni functions.
   *
   * \author Kripasindhu Sarkar
   *
   */
  template<typename PointT>
  class Detector3D: public Detector
  {
  public:
    Detector3D(const std::string & trained_data_location) : Detector(trained_data_location) {}

    /** \brief Function for performing detection on a segmented scene.
     * The purpose of this function is to perform detection on a segmented scene or an 'object candidate'. i.e. the entire scene is considered as an 'object' or an detection. It is possible for a scene to trigger multiple detections.
     * \param[in] scene An instance of 3D scene
     * \return A number of detections as an Detections instance.
    */
    virtual shared_ptr<Detections> detect(shared_ptr<ScenePointCloud<PointT> > scene) = 0;

    /** \brief Function for performing detection on an entire scene.
     *  The purpose of this function is to detect an object in an entire scene. Thus, other than the type of detection we also have information about the location of the detection w.r.t. the scene.
     * \param[in] scene An instance of 3D scene
     * \return A number of detections as an Detections3D instance containing information about the detection and its 3D pose.
    */
    virtual shared_ptr<Detections3D> detectOmni(shared_ptr<ScenePointCloud<PointT> > scene) = 0;
  };

  /** \brief The detector of 2D scene performing a 'complete detection'.
   *
   * This class takes a 2D scene (SceneImage) as input and performs complete detection on them. That is, other than finding the bounding box or location of the object in the image it
   * finds out the 3D location and orientation (in other words translation and rotation) of the object in the actual 3D scene as well.
   *
   * \author Kripasindhu Sarkar
   *
   */
  class Detector2DComplete: public Detector
  {
  public:
    Detector2DComplete(const std::string & trained_data_location) : Detector(trained_data_location) {}

    /** \brief Function for performing detection on a segmented scene.
     * The purpose of this function is to perform detection on a segmented scene or an 'object candidate'. i.e. the entire scene is considered as an 'object' or an detection. It is possible for a scene to trigger multiple detections.
     * \param[in] scene An instance of 2D scene
     * \return A number of detections as an Detections instance.
    */
    virtual shared_ptr<Detections> detect(shared_ptr<SceneImage> scene) = 0;

    /** \brief Function for performing detection on an entire scene.
     *  The purpose of this function is to detect an object in an entire scene. Thus, other than the type of detection we also have information about the location of the detection w.r.t. the scene.
     * \param[in] scene An instance of 2D scene
     * \return A number of detections as an Detections3D instance containing information about the detection and its pose in 3D.
    */
    virtual shared_ptr<Detections3D> detectOmni(shared_ptr<SceneImage> scene) = 0;
  };

}
