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
*///
// Created by sarkar on 16.07.15.
//

#include "od/detectors/global2D/FaceRecognizer.h"

namespace od
{
  namespace g2d
  {

    FaceRecognizer::FaceRecognizer(FaceRecogType recog_type, int num_components, double threshold): 
                                       recog_type_(recog_type), num_components_(num_components), threshold_(threshold), 
                                       im_height_(0), im_width_(0)
    {
      trained_data_identifier_ = std::string("FACERECOG");
      trained_data_ext_ = std::string("facerec.xml");
    }

    void FaceRecognizer::init()
    {
      switch(recog_type_)
      {
        case _FACE_FISCHER:
          cv_recognizer_ = cv::face::createFisherFaceRecognizer(num_components_, threshold_);
          break;
        case _FACE_EIGEN:
          cv_recognizer_ = cv::face::createEigenFaceRecognizer(num_components_, threshold_);
          break;
        default:
          std::cout << "FATAL: FACETYPE NOT FOUND!";
      }
    }

    void FaceRecognizer::initTrainer()
    {
      init();
    }

    void FaceRecognizer::initDetector()
    {
      if(!trained_)
      {
        init();

        //get models in the directory
        std::vector<std::string> files;
        fileutils::getFilesInDirectoryRec(getSpecificTrainingDataLocation(), trained_data_ext_, files);

        if(files.size() == 0)
        {
          std::cout << "FATAL: Trained data not found" << std::endl;
          exit(1);
        }

        //choose the first
        cv_recognizer_->load(files[0]);
      }
    }

    int FaceRecognizer::train()
    {
      std::vector<cv::Mat> images;
      std::vector<int> labels;
      try
      {
        read_csv(training_input_location_, images, labels);
      } catch(cv::Exception & e)
      {
        std::cerr << "Error opening file \"" << training_input_location_ << "\". Reason: " << e.msg << std::endl;
        return -1;
      }
      cv_recognizer_->train(images, labels);
      fileutils::createTrainingDir(getSpecificTrainingDataLocation());

      cv_recognizer_->save(getSpecificTrainingDataLocation() + "/trained." + trained_data_ext_);
      trained_ = true;

      //the training set has atleast one image
      im_width_ = images[0].cols;
      im_height_ = images[0].rows;

      return 0;
    }

    shared_ptr<Detection> FaceRecognizer::detect(shared_ptr<Scene> scene) 
    {
      std::cout << "not implemented, use with shared_ptr<SceneImage>" <<std::endl; 
      return nullptr;
    };

    shared_ptr<Detections> FaceRecognizer::detectOmni(shared_ptr<Scene> scene)
    {
      std::cout << "not implemented, use with shared_ptr<SceneImage>" <<std::endl; 
      return nullptr;
    };

    int FaceRecognizer::detect(shared_ptr<Scene> scene, const std::vector<shared_ptr<Detection> > & detections)
    {
      std::cout << "not implemented, use with shared_ptr<SceneImage>" <<std::endl; 
      return -1;
    }

    shared_ptr<Detections> FaceRecognizer::detectOmni(shared_ptr<SceneImage> scene) 
    {
      std::cout << "not implemented, use detect()" <<std::endl; 
      return nullptr;
    };


    shared_ptr<Detections> FaceRecognizer::detect(shared_ptr<SceneImage> scene)
    {
      cv::Mat face_edited;
      cv::cvtColor(scene->getCVImage(), face_edited, CV_BGR2GRAY);

      if(trained_)
      {
        cv::resize(face_edited.clone(), face_edited, cv::Size(im_width_, im_height_), 1.0, 1.0, cv::INTER_CUBIC);
      }

      int label = 100;
      double confidence;
      cv_recognizer_->predict(face_edited, label, confidence);

      //fill in the detection
      shared_ptr<Detection2D> detection(new Detection2D(detection::CLASSIFICATION, std::to_string(label), confidence));
      shared_ptr<Detections2D> detections = make_shared<Detections2D>();
      detections->push_back(detection);
      
      return detections;
    }


    void FaceRecognizer::read_csv(const std::string & filename, std::vector<cv::Mat> & images, std::vector<int> & labels, 
                                    const std::string & separator)
    {
      std::ifstream file(filename.c_str(), std::ifstream::in);
      if(!file)
      {
        std::string error_message("No valid input file was given, please check the given filename.");
        CV_Error(CV_StsBadArg, error_message);
      }
      std::string line, path, classlabel;
      while(getline(file, line))
      {
        std::stringstream liness(line);
        getline(liness, path, *separator.c_str());
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty())
        {
          images.push_back(cv::imread(path, 0));
          labels.push_back(atoi(classlabel.c_str()));
        }
      }
    }

    const FaceRecognizer::FaceRecogType & FaceRecognizer::getRecogtype() const
    {
      return recog_type_;
    }

    void FaceRecognizer::setRecogtype(const FaceRecognizer::FaceRecogType & recog_type)
    {
      recog_type_ = recog_type;
    }

    int FaceRecognizer::getThreshold() const
    {
      return threshold_;
    }

    void FaceRecognizer::setThreshold(int threshold)
    {
      threshold_ = threshold;
    }

    int FaceRecognizer::getNumComponents() const
    {
      return num_components_;
    }

    void FaceRecognizer::setNumComponents(int num_components)
    {
      num_components_ = num_components;
    }
    
  }
}