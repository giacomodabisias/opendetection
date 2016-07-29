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
// Created by sarkar on 03.06.15.
//
#pragma once
#include <string>
#include <vector>
#include "od/common/pipeline/Detection.h"
#include "od/common/pipeline/Scene.h"

namespace od
{

  enum DetectionMethod {
    PC_GLOBAL_FEATUREMATCHING,
    PC_LOCAL_CORRESPONDENCE_GROUPING,
    IMAGE_LOCAL_SIMPLE,
    IMAGE_GLOBAL_DENSE,
    IMAGE_GLOBAL_CLASSIFICATION,
  };

  /** \brief The common class for detectors. Both Trainers and Detectors drerives from this and therefore, all the common data/functionalities of Trainers and Detectors should go here.
  *
  *
  * \author Kripasindhu Sarkar
  *
  */

  class DetectorCommon
  {
  public:

    DetectorCommon(const std::string & trained_data_location);
    DetectorCommon() {}

    virtual void init() = 0;


    /** \brief Gets/Sets the directory containing the data for training. The trainer uses the data from directory for training. Detectors can use this location to get additional information in its detection algirhtms as well.
      */
    std::string getTrainingInputLocation() const;

    /** \brief Gets/Sets the directory containing the data for training. The trainer uses the data from directory for training. Detectors can use this location to get additional information in its detection algirhtms as well.
      */
    void setTrainingInputLocation(const std::string & training_input_location);

    /** \brief Gets/Sets the base directory for trained data. This should be same for all Trainers and Detectors and can be considered as the 'database' of trained data. Trainers uses one of its
     * subdirectories based on its type to store algo specific trained data. The corresponding Detector would use the same directory to fetch the trained data for online detection.
      */
    std::string getTrainedDataLocation() const;


    /** \brief The base directory for trained data. This should be same for all Trainers and Detectors and can be considered as the 'database' of trained data. Trainers uses one of its
     * subdirectories based on its type to store algo specific trained data. The corresponding Detector would use the same directory to fetch the trained data for online detection.
     */
    virtual void setTrainedDataLocation(const std::string & trained_data_location);


    /** \brief Gets the specific directory for a Trainer or a Detector inside trained_data_location_.
      */
    std::string getSpecificTrainingDataLocation();

    std::string getSpecificTrainingData();

    const std::string & getTrainedDataID() const;

    void setTrainedDataID(const std::string & trainedDataID);

  protected:

    std::string training_input_location_, trained_data_location_;
    std::string trained_data_id_, trained_location_identifier_;

  };

  /** \brief This is the main class for object detection and recognition.
   *
   *
   * \author Kripasindhu Sarkar
   *
   */
  class ObjectDetector {
  public:

    const DetectionMethod & getMethod() const;

    void setDetectionMethod(const DetectionMethod & detection_method);

    bool getAlwaysTrain() const;
    void setAlwaysTrain(bool always_train);

    std::string getTrainingInputLocation() const;
    void setTrainingInputLocation(const std::string & training_input_location);

    std::string getTrainingDataLocation() const;
    void setTrainingDataLocation(const std::string & training_data_location);

    std::string getSpecificTrainingDataLocation();

    virtual void init() = 0;

    virtual void initDetector(){}
    virtual void initTrainer(){}

    virtual int train() = 0;

    virtual int detect(shared_ptr<Scene> scene, const std::vector<shared_ptr<Detection> > & detections) = 0;

    virtual shared_ptr<Detection> detect(shared_ptr<Scene> scene) = 0;
    virtual shared_ptr<Detections> detectOmni(shared_ptr<Scene> scene) = 0;

  protected:

    DetectionMethod method_;
    bool always_train_;
    bool trained_;
    std::string training_input_location_, training_data_location_;
    std::string trained_data_ext_, trained_data_identifier_;
  };

}
