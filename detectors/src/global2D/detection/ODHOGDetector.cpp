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
*///
// Created by sarkar on 15.07.15.
//

#include "od/detectors/global2D/detection/ODHOGDetector.h"

using namespace std;

namespace od
{
  namespace g2d
  {

    ODHOGDetector::ODHOGDetector(const std::string & trained_data_location_ = "", const cv::Size & win_size = cv::Size(64,128),
                  const cv::Size & block_size = cv::Size(16,16), const cv::Size & block_stride = cv::Size(8,8), const cv::Size & cell_size = cv::Size(8,8),
                  float hit_threshold = 0.0)
    {
      TRAINED_LOCATION_DENTIFIER_ = "HOG";
      TRAINED_DATA_ID_ = "hog.xml";
      metainfo_ = true;
      svm_type_ = OD_DEFAULT_PEOPLE;

      if (trained_data_location_ != "")
        svm_type_ = OD_FILE;
    }

    void ODHOGDetector::init()
    {

      switch(svm_type_)
      {
        case OD_DEFAULT_PEOPLE:
          hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
          cout << "HOG TYPE: OpenCV Default People" << endl;
          //hog_.save(getSpecificTrainingDataLocation() + "/defaultpeople." + TRAINED_DATA_EXT_);
          break;
        case OD_DAIMLER_PEOPLE:
          hog_.win_size = cv::Size(48, 96);
          hit_threshold = 1.2;
          hog_.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
          cout << "HOG TYPE: OpenCV Daimler People" << endl;
          //hog_.save(getSpecificTrainingDataLocation() + "/daimlerpeople." + TRAINED_DATA_EXT_);
          break;
        case OD_FILE:
          string hogfile = FileUtils::getFirstFile(getSpecificTrainingDataLocation(), TRAINED_DATA_ID_);
          load(hogfile);
          cout << "HOG TYPE: Custom HOG features loaded from: " << hogfile << endl;
          break;
          //dont set anything for custom, it is to be set by the user by setSVMDetector
      }

      printParameters();
    }

    void ODHOGDetector::load(const std::string & file_name)
    {
      cv::FileStorage fs(file_name, cv::FileStorage::READ);
      fs["hitThreshold"] >> hit_threshold_;
      cv::FileNode fn = fs[cv::FileStorage::getDefaultObjectName(file_name)];
      hog_.read(fn);
    }

    void ODHOGDetector::setSVMDetector(std::vector<float> svm_detector)
    {
      hog_.setSVMDetector(svm_detector);
    }

    ODDetections2D * ODHOGDetector::detectOmni(ODSceneImage * scene)
    {
      //always create a detection
      ODDetections2D * detections = new ODDetections2D;

      vector<cv::Rect> found, found_filtered;
      hog_.detectMultiScale(scene->getCVImage(), found, hit_threshold, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);

      cv::Mat viz = scene->getCVImage().clone();
      for(int i = 0; i < found.size(); i++)
      {
        ODDetection2D * detection2D = new ODDetection2D;
        detection2D->setBoundingBox(found[i]);
        detection2D->setId("PEOPLE");
        detection2D->setType(ODDetection::OD_DETECTION_CLASS);
        detections->push_back(detection2D);

        if(metainfo_)
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

    ODDetections * ODHOGDetector::detect(ODSceneImage * scene)
    {
      //always create a detection
      ODDetections * detections = new ODDetections;

      cv::Mat scaled_window;
      cv::resize(scene->getCVImage(), scaled_window, hog_.win_size);

      std::vector<cv::Point> found_locations;

      hog_.detect(scene->getCVImage(), found_locations, hitThreshold);
      if (!found_locations.empty())
      {
        ODDetection2D * detection2D = new ODDetection2D;
        detection2D->setId("PEOPLE");
        detection2D->setType(ODDetection::OD_DETECTION_CLASS);
        detections->push_back(detection2D);
      }

      return detections;
    }

    void ODHOGDetector::setTrainedDataLocation(const std::string & trained_data_location)
    {
      trained_data_location_ = trained_data_location;
      svmtype_ = OD_FILE;
    }

    const SVMType & ODHOGDetector::getSvmtype() const
    {
      return svm_type_;
    }

    void ODHOGDetector::setSvmtype(const SVMType & svm_type)
    {
      svm_type_ = svm_type;
    }

    const cv::Size & ODHOGDetector::getWinSize() const
    {
      return win_size;
    }

    void ODHOGDetector::setWinSize(const cv::Size & win_size)
    {
      win_size_ = win_size;
    }

    const cv::Size & ODHOGDetector::getBlockSize() const
    {
      return block_size;
    }

    void ODHOGDetector::setBlockSize(const cv::Size & block_size)
    {
      block_size_ = block_size;
    }

    const cv::Size & getBlockStride() const
    {
      return block_stride;
    }

    void ODHOGDetector::setBlockStride(const cv::Size & block_stride)
    {
      ODHOGDetector::blockStride = blockStride;
    }

    const cv::Size & ODHOGDetector::getCellSize() const
    {
      return cell_size;
    }

    void ODHOGDetector::setCellSize(const cv::Size & cell_size)
    {
      ODHOGDetector::cellSize = cellSize;
    }

    float ODHOGDetector::getHitThreshold() const
    {
      return hit_threshold;
    }

    void ODHOGDetector::setHitThreshold(float hit_threshold)
    {
      hit_threshold_ = hit_threshold;
    }

    void ODHOGDetector::setSVMFromFile(const std::string & file_name)
    {
      vector<float> descriptor_vector;
      printf("Reading descriptor vector from file '%s'\n", file_name.c_str());

      ifstream file;
      float percent;
      file.open(file_name.c_str(), ios::in);
      if (file.good() && file.is_open()) {

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

    void ODHOGDetector::printParameters()
    {
      cout << "winSize: " << win_size_ << endl;
      cout << "blockSize: " << block_size_ << endl;
      cout << "blockStride: " << block_stride_ << endl;
      cout << "cellSize: " << cell_size_ << endl;
      cout << "hitThreshold: " << hit_threshold_ << endl;
    }
  }
}
