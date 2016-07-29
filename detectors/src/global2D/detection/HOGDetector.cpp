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
// Created by sarkar on 15.07.15.
//

#include "od/detectors/global2D/detection/HOGDetector.h"

namespace od
{
  namespace g2d
  {

    HOGDetector::HOGDetector(const std::string & trained_data_location_, const cv::Size & win_size, const cv::Size & block_size, 
                                 const cv::Size & block_stride, const cv::Size & cell_size, float hit_threshold): 
                                  Detector2D(trained_data_location_),  win_size_(win_size), block_size_(block_size), block_stride_(block_stride),
                                  cell_size_(cell_size), hit_threshold_(hit_threshold), hog_(win_size, block_size, block_stride, cell_size, 9, 1, -1,
                                  cv::HOGDescriptor::L2Hys, 0.2, false, cv::HOGDescriptor::DEFAULT_NLEVELS)
    {
      trained_location_identifier_ = std::string("HOG");
      trained_data_id_ = std::string("hog.xml");
      meta_info_ = true;
      svm_type_ = _DEFAULT_PEOPLE;

      if(trained_data_location_ != std::string(""))
        svm_type_ = _FILE;
    }

    void HOGDetector::init()
    {

      switch(svm_type_)
      {
        case _DEFAULT_PEOPLE:
          hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
          std::cout << "HOG TYPE: OpenCV Default People" << std::endl;
          //hog_.save(getSpecificTrainingDataLocation() + "/defaultpeople." + TRAINED_DATA_EXT_);
          break;
        case _DAIMLER_PEOPLE:
          hog_.winSize = cv::Size(48, 96);
          hit_threshold_ = 1.2;
          hog_.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
          std::cout << "HOG TYPE: OpenCV Daimler People" << std::endl;
          //hog_.save(getSpecificTrainingDataLocation() + "/daimlerpeople." + TRAINED_DATA_EXT_);
          break;
        case _FILE:
          std::string hogfile = fileutils::getFirstFile(getSpecificTrainingDataLocation(), trained_data_id_);
          load(hogfile);
          std::cout << "HOG TYPE: Custom HOG features loaded from: " << hogfile << std::endl;
          break;
          //dont set anything for custom, it is to be set by the user by setSVMDetector
      }

      printParameters();
    }

    void HOGDetector::load(const std::string & file_name)
    {
      cv::FileStorage fs(file_name, cv::FileStorage::READ);
      fs["hitThreshold"] >> hit_threshold_;
      cv::FileNode fn = fs[cv::FileStorage::getDefaultObjectName(file_name)];
      hog_.read(fn);
    }

    void HOGDetector::setSVMDetector(std::vector<float> svm_detector)
    {
      hog_.setSVMDetector(svm_detector);
    }

    shared_ptr<Detections2D> HOGDetector::detectOmni(shared_ptr<SceneImage> scene)
    {
      //always create a detection
      shared_ptr<Detections2D> detections = make_shared<Detections2D>();

      std::vector<cv::Rect> found;
      hog_.detectMultiScale(scene->getCVImage(), found, hit_threshold_, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);

      cv::Mat viz = scene->getCVImage().clone();
      for(size_t i = 0; i < found.size(); ++i)
      {
        shared_ptr<Detection2D> detection2D = make_shared<Detection2D>();
        detection2D->setBoundingBox(found[i]);
        detection2D->setId("PEOPLE");
        detection2D->setType(od::Detection::CLASSIFICATION);
        detections->push_back(detection2D);

        if(meta_info_)
        {
          cv::Rect r = found[i];
          r.x += cvRound(r.width * 0.1);
          r.width = cvRound(r.width * 0.8);
          r.y += cvRound(r.height * 0.06);
          r.height = cvRound(r.height * 0.9);
          rectangle(viz, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
        }
      }
      detections->setMetainfoImage(viz);

      return detections;
    }

    shared_ptr<Detections> HOGDetector::detect(shared_ptr<SceneImage> scene)
    {
      //always create a detection
      shared_ptr<Detections> detections = make_shared<Detections>();

      cv::Mat scaled_window;
      cv::resize(scene->getCVImage(), scaled_window, hog_.winSize);

      std::vector<cv::Point> found_locations;

      hog_.detect(scene->getCVImage(), found_locations, hit_threshold_);
      if(!found_locations.empty())
      {
        shared_ptr<Detection2D> detection2D = make_shared<Detection2D>();
        detection2D->setId("PEOPLE");
        detection2D->setType(od::Detection::CLASSIFICATION);
        detections->push_back(detection2D);
      }

      return detections;
    }

    void HOGDetector::setTrainedDataLocation(const std::string & trained_data_location)
    {
      trained_data_location_ = trained_data_location;
      svm_type_ = _FILE;
    }

    const HOGDetector::SVMType & HOGDetector::getSvmtype() const
    {
      return svm_type_;
    }

    void HOGDetector::setSvmtype(const HOGDetector::SVMType & svm_type)
    {
      svm_type_ = svm_type;
    }

    const cv::Size & HOGDetector::getWinSize() const
    {
      return win_size_;
    }

    void HOGDetector::setWinSize(const cv::Size & win_size)
    {
      win_size_ = win_size;
    }

    const cv::Size & HOGDetector::getBlockSize() const
    {
      return block_size_;
    }

    void HOGDetector::setBlockSize(const cv::Size & block_size)
    {
      block_size_ = block_size;
    }

    const cv::Size & HOGDetector::getBlockStride() const
    {
      return block_stride_;
    }

    void HOGDetector::setBlockStride(const cv::Size & block_stride)
    {
      block_stride_ = block_stride;
    }

    const cv::Size & HOGDetector::getCellSize() const
    {
      return cell_size_;
    }

    void HOGDetector::setCellSize(const cv::Size & cell_size)
    {
      cell_size_ = cell_size;
    }

    float HOGDetector::getHitThreshold() const
    {
      return hit_threshold_;
    }

    void HOGDetector::setHitThreshold(float hit_threshold)
    {
      hit_threshold_ = hit_threshold;
    }

    void HOGDetector::setSVMFromFile(const std::string & file_name)
    {
      std::vector<float> descriptor_vector;
      std::cout << "Reading descriptor vector from file " << file_name << std::endl;;

      std::ifstream file;
      file.open(file_name.c_str(), std::ios::in);
      if(file.good() && file.is_open()) {

        double d;
        while(file >> d)
        {
          //cout << d << " ";
          descriptor_vector.push_back(d);
        }
        file.close();
      }

      hog_.setSVMDetector(descriptor_vector);
    }

    void HOGDetector::printParameters()
    {
      std::cout << "winSize: " << win_size_ << std::endl;
      std::cout << "blockSize: " << block_size_ << std::endl;
      std::cout << "blockStride: " << block_stride_ << std::endl;
      std::cout << "cellSize: " << cell_size_ << std::endl;
      std::cout << "hitThreshold: " << hit_threshold_ << std::endl;
    }

    shared_ptr<Detections> HOGDetector::detectOmni(shared_ptr<Scene> scene) 
    {
      return nullptr;
    }
  }
}
