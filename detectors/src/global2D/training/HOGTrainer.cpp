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
// Created by sarkar on 13.08.15.
//


#include "od/detectors/global2D/training/HOGTrainer.h"

namespace od
{
  namespace g2d
  {

    HOGTrainer::HOGTrainer(const std::string & training_input_location_, const std::string & trained_data_location_, const cv::Size & win_size, 
                               const cv::Size & block_size, const cv::Size & block_stride, const cv::Size & cell_size, float hit_threshold):
                               Trainer(training_input_location_, trained_data_location_),  win_size_(win_size), block_size_(block_size), 
                               block_stride_(block_stride), cell_size_(cell_size), hog_(win_size, block_size, block_stride, cell_size, 9)
    {

      trained_location_identifier_ = std::string("HOG");
      trained_data_id_ = std::string("hog.xml");

      //algo parameter init
      training_padding_ = cv::Size(0, 0);
      start_hog_pos_ = cv::Point(15, 15);
      no_features_neg_ = 10;
      win_stride_ = cv::Size();
      train_hard_negative_ = false;

      if(trained_data_location_ != std::string(""))
      {
        pos_samples_dir_ = training_input_location_ + std::string("/pos");
        neg_samples_dir_ = training_input_location_ + std::string("/neg");
      }


      //internal data
      fileutils::createTrainingDir(getSpecificTrainingDataLocation());
      features_file_ = getSpecificTrainingDataLocation() + std::string("/features.dat");
      svm_model_file_ = getSpecificTrainingDataLocation() + std::string("/svmlightmodel.dat");
      svm_model_hard_ = getSpecificTrainingDataLocation() + std::string("/svmlightmodelhard.dat");
      descriptor_vector_file_ = getSpecificTrainingDataLocation() + std::string("/descriptorvector.dat");
      descriptor_vector_file_ = getSpecificTrainingDataLocation() + std::string("/descriptorvectorHard.dat");

    }

    void HOGTrainer::saveDescriptorVectorToFile(const std::vector<float> & descriptor_vector, const std::string & file_name)
    {
      std::cout << "Saving descriptor vector to file " << file_name << std::endl;
      std::fstream file;
      float percent;
      file.open(file_name.c_str(), std::ios::out);
      const unsigned int descriptors_num = descriptor_vector.size();

      if(file.good() && file.is_open())
      {
        std::cout << "Saving " << descriptors_num << " descriptor vector features:\t" << std::endl;
        for(size_t feature = 0; feature < descriptors_num; ++feature)
        {
          if((feature % 10 == 0) || (feature == (descriptors_num - 1)))
          {
            percent = ((1 + feature) * 100 / descriptors_num);
            std::cout << feature << "(" <<percent << "%)" << "\xd";
          }
          file << descriptor_vector[feature] << " ";
        }
        std::cout << std::endl;
        file << std::endl;
        file.flush();
        file.close();
      }
    }

    void HOGTrainer::calculateFeaturesFromInput(const std::string & image_file_name, std::vector<float> & feature_vector, cv::HOGDescriptor & hog)
    {

      cv::Mat image_data_orig = cv::imread(image_file_name, 0);

      if(image_data_orig.empty())
      {
        feature_vector.clear();
        std::cout << "Error: HOG image " << image_file_name << " is empty, features calculation skipped!" << std::endl;
        return;
      }

      std::vector<cv::Point> locations;
      locations.push_back(start_hog_pos_);
      hog.compute(image_data_orig, feature_vector, win_stride_, training_padding_, locations);
      //cout << "Desc size :" << featureVector.size();
      //cout << ": Expected size :" << hog.getDescriptorSize() << endl;
    }

    void HOGTrainer::detectTrainingSetTest(const cv::HOGDescriptor & hog, double hit_threshold, const std::vector<std::string> & pos_file_names, 
                                             const std::vector<std::string> & neg_file_names)
    {
      unsigned int true_positives = 0;
      unsigned int true_negatives = 0;
      unsigned int false_positives = 0;
      unsigned int false_negatives = 0;

      std::vector<cv::Point> found_detection;
      cv::Mat image_data;
      // Walk over positive training samples, generate images and detect
      for(auto & pf : pos_file_names)
      {
        image_data = cv::imread(pf, 0);
        hog.detect(image_data, found_detection, hit_threshold, win_stride_, training_padding_);
        if(found_detection.size() > 0)
        {
          ++true_positives;
          false_negatives += found_detection.size() - 1;
        } else
        {
          ++false_negatives;
        }
      }
      // Walk over negative training samples, generate images and detect
      for(auto & nf : neg_file_names)
      {
        image_data = cv::imread(nf, 0);
        hog.detect(image_data, found_detection, hit_threshold, win_stride_, training_padding_);
        if(found_detection.size() > 0)
        {
          false_positives += found_detection.size();
        } else
        {
          ++true_negatives;
        }
      }

      std::cout << "Results: " << std::endl << "\tTrue Positives: " << true_positives << std::endl << "\tTrue Negatives: "
                << true_negatives << "\tFalse Positives: " << false_positives << std::endl << " \tFalse Negatives:" << false_negatives << std::endl;
    }


    void HOGTrainer::calculateFeaturesFromImageLoc(const cv::Mat & image_data, std::vector<float> & feature_vector, const cv::HOGDescriptor & hog, 
                                                     const cv::Point & start_pos)
    {

      std::vector<cv::Point> locations;
      locations.push_back(start_pos);
      hog.compute(image_data, feature_vector, win_stride_, training_padding_, locations);
    }

    void HOGTrainer::handleNegetivefile(const std::string & image_file_name, cv::HOGDescriptor & hog, std::fstream & file)
    {

      cv::Mat image_data = cv::imread(image_file_name, 0);

      //cout << "Image size : " << imageData.size() << "random points : " << endl;
      //get hog at random 10 image location
      for(size_t i = 0; i < no_features_neg_; ++i)
      {
        cv::Point feat_loc(rand() % (image_data.cols - win_size_.width), rand() % (image_data.rows - win_size_.height));
        //cout << feat_loc << endl;

        //use this p for finding feature
        std::vector<float> feature_vector;
        calculateFeaturesFromImageLoc(image_data, feature_vector, hog, feat_loc);

        if(!feature_vector.empty())
        {
          /* Put positive or negative sample class to file,
       * true=positive, false=negative,
       * and convert positive class to +1 and negative class to -1 for SVMlight
       */
          file << "-1";
          // Save feature vector components
          for(size_t feature = 0; feature < feature_vector.size(); ++feature)
          {
            file << " " << (feature + 1) << ":" << feature_vector[feature];
          }
          file << std::endl;
        }
      }

    }

    void HOGTrainer::createHardTrainingData(const cv::HOGDescriptor & hog, double hit_threshold, const std::vector<std::string> & neg_file_names)
    {
      std::fstream file;
      file.open(features_file_.c_str(), std::fstream::app);
      if(!file.is_open())
      {
        std::cout << "ERROR opening previous feature file! HARD training failed!" << std::endl;
      }
      std::cout << "Appending HARD negetive features to " + features_file_ << std::endl;
      //cvNamedWindow("hardneg", WINDOW_AUTOSIZE);

      // Walk over negative training samples, generate images and detect
      int counter = 0;
      for(auto & nf :  neg_file_names)
      {
        const cv::Mat image_data = cv::imread(nf, 0);

        std::vector<cv::Rect> found_locations;
        hog.detectMultiScale(image_data, found_locations, hit_threshold);

        for(size_t i = 0; i < found_locations.size(); ++i)
        {
          counter++;
          //cout << "## hard examples ## FALSE POS FOUND : including the descriptor in training set" << endl;

          cv::Mat neg_img(image_data(found_locations[i]));
          //imshow("hardneg", negimg); waitKey(2000);

          cv::Mat resized_neg;
          resize(neg_img, resized_neg, win_size_);
          // imshow("hardneg", resized_neg); waitKey(4000);

          std::vector<float> feature_vector;
          calculateFeaturesFromImageLoc(resized_neg, feature_vector, hog, cv::Point(0, 0));

          if(!feature_vector.empty())
          {
            file << "-1";
            for(size_t feature = 0; feature < feature_vector.size(); ++feature)
            {
              file << " " << (feature + 1) << ":" << feature_vector.at(feature);
            }
            file << std::endl;
          }
        }
      }
      std::cout << "Wrote " << counter << " HARD negetive features" << std::endl;

      //CLOSE THE APPENDED DATA FILE
      file.close();
    }

    double HOGTrainer::trainWithSVMLight(const std::string & svm_model_file, const std::string & svm_descriptor_file, 
                                           std::vector<float> & descriptor_vector)
    {
      SVMlight * svmlight= SVMlight::getInstance();
      //training takes featurefile as input, produces hitthreshold and vector as output
      std::cout << "Calling " << svmlight->getSVMName() << std::endl;
      svmlight->read_problem(const_cast<char *> (features_file_.c_str()));
      svmlight->train(); // Call the core libsvm training procedure

      std::cout << "Training done, saving model file!"<< std::endl;
      svmlight->saveModelToFile(svm_model_file);

      std::cout << "Generating representative single HOG feature vector using svmlight!" << std::endl;
      descriptor_vector.resize(0);
      // Generate a single detecting feature vector (v1 | b) from the trained support vectors, for use e.g. with the HOG algorithm
      svmlight->getSingleDetectingVector(descriptor_vector);
      // And save the precious to file system
      saveDescriptorVectorToFile(descriptor_vector, svm_descriptor_file);
      // Detector detection tolerance threshold
      hit_threshold_ = svmlight->getThreshold();
      return hit_threshold_;
    }

    int HOGTrainer::train()
    {

      std::vector<std::string> positive_training_images;
      std::vector<std::string> negative_training_images;
      std::vector<std::string> valid_extensions;

      valid_extensions.push_back(".jpg");
      valid_extensions.push_back(".png");
      valid_extensions.push_back(".ppm");

      fileutils::getFilesInDirectoryRec(pos_samples_dir_, valid_extensions, positive_training_images);
      fileutils::getFilesInDirectoryRec(neg_samples_dir_, valid_extensions, negative_training_images);

      unsigned int pos_size = positive_training_images.size();
      unsigned int neg_size = negative_training_images.size();
      std::cout << "No of positive Training Files: " << pos_size << std::endl;
      std::cout << "No of neg Training Files: "<< neg_size << std::endl;

      if(pos_size + neg_size == 0)
      {
        std::cout << "No training sample files found, nothing to do!" << std::endl;
        return 0;
      }

      /// @WARNING: This is really important, some libraries (e.g. ROS) seems to set the system locale 
      // which takes decimal commata instead of points which causes the file input parsing to fail
      setlocale(LC_ALL, "C"); // Do not use the system locale
      setlocale(LC_NUMERIC, "C");
      setlocale(LC_ALL, "POSIX");

      std::cout << "Reading files, generating HOG features and save them to file: " << features_file_ << std::endl;

      std::fstream file;
      file.open(features_file_.c_str(), std::fstream::out);
      if(file.is_open())
      {

        // Iterate over POS IMAGES
        for(auto & pt : positive_training_images)
        {

          std::vector<float> feature_vector;
          calculateFeaturesFromInput(pt, feature_vector, hog_);
          if(!feature_vector.empty())
          {
            /* Put positive or negative sample class to file,
            * true=positive, false=negative,
            * and convert positive class to +1 and negative class to -1 for SVMlight
            */
            file << "+1";
            // Save feature vector components
            for(unsigned int feature = 0; feature < feature_vector.size(); ++feature)
            {
              file << " " << (feature + 1) << ":" << feature_vector[feature];
            }
            file << std::endl;
          }
        }


        // Iterate over NEG IMAGES
        for(auto & ni : negative_training_images)
        {
          handleNegetivefile(ni, hog_, file);
        }

        std::cout << std::endl;
        file.flush();
        file.close();

      } else
      {
        std::cout << "Error opening file " << features_file_;
        return -1;
      }


      //train them with SVM
      std::vector<float> descriptor_vector;
      hit_threshold_ = trainWithSVMLight(svm_model_file_, descriptor_vector_file_, descriptor_vector);

      // Pseudo test our custom detecting vector
      hog_.setSVMDetector(descriptor_vector);
      std::cout << "Testing training phase using training set as test set (just to check if training is ok - no detection quality conclusion with this!)" << std::endl;
      detectTrainingSetTest(hog_, hit_threshold_, positive_training_images, negative_training_images);

      if(train_hard_negative_)
      {
        std::cout << "Preparing for training HARD negetive windows" << std::endl;
        //create hard training examples
        createHardTrainingData(hog_, hit_threshold_, negative_training_images);
        //train again
        hit_threshold_ = trainWithSVMLight(svm_model_hard_, descriptor_vector_hard_, descriptor_vector);

        // Pseudo test our custom detecting vector
        hog_.setSVMDetector(descriptor_vector);
        std::cout << "Testing training phase using training set as test set after HARD EXAMPLES (just to check if training is ok - no detection quality conclusion with this!)" << std::endl;
        detectTrainingSetTest(hog_, hit_threshold_, positive_training_images, negative_training_images);
      }

      save(getSpecificTrainingDataLocation() + "/odtrained." + trained_data_id_);

      return 0;
    }


    void HOGTrainer::readDescriptorsFromFile(const std::string & file_name, std::vector<float> & descriptor_vector)
    {
      std::cout << "Reading descriptor vector from file " << file_name << std::endl;

      std::ifstream file;
      file.open(file_name.c_str(), std::ios::in);
      if(file.good() && file.is_open())
      {
        double d;
        while(file >> d)
        {
          descriptor_vector.push_back(d);
        }
        file.close();
      }
    }

    void HOGTrainer::save(const std::string & file_name)
    {
      cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
      fs << "hitThreshold" << hit_threshold_;
      hog_.write(fs, cv::FileStorage::getDefaultObjectName(file_name));
    }

    const std::string & HOGTrainer::getPosSamplesDir() const
    {
      return pos_samples_dir_;
    }

    void HOGTrainer::setPosSamplesDir(const std::string & pos_samples_dir)
    {
      pos_samples_dir_ = pos_samples_dir;
    }

    const std::string & HOGTrainer::getNegSamplesDir() const
    {
      return neg_samples_dir_;
    }

    void HOGTrainer::setNegSamplesDir(const std::string & neg_samples_dir)
    {
      neg_samples_dir_ = neg_samples_dir;
    }

    int HOGTrainer::getNOFeaturesNeg() const
    {
      return no_features_neg_;
    }

    void HOGTrainer::setNOFeaturesNeg(int featno)
    {
      no_features_neg_ = featno;
    }

    const cv::Point & HOGTrainer::getStartHogPos() const
    {
      return start_hog_pos_;
    }

    void HOGTrainer::setStartHogPos(const cv::Point & start_hog_pos)
    {
      start_hog_pos_ = start_hog_pos;
    }

    const cv::Size & HOGTrainer::getWinSize() const
    {
      return win_size_;
    }

    void HOGTrainer::setWinSize(const cv::Size & win_size)
    {
      win_size_ = win_size;
    }

    const cv::Size & HOGTrainer::getBlockSize() const
    {
      return block_size_;
    }

    void HOGTrainer::setBlockSize(const cv::Size & block_size)
    {
      block_size_ = block_size;
    }

    const cv::Size & HOGTrainer::getBlockStride() const
    {
      return block_stride_;
    }

    void HOGTrainer::setBlockStride(const cv::Size & block_stride)
    {
      block_stride_ = block_stride;
    }

    const cv::Size & HOGTrainer::getCellSize() const
    {
      return cell_size_;
    }

    void HOGTrainer::setCellSize(const cv::Size & cell_size)
    {
      cell_size_ = cell_size;
    }

    const cv::Size & HOGTrainer::getTrainingPadding() const
    {
      return training_padding_;
    }

    void HOGTrainer::setTrainingPadding(const cv::Size & training_padding)
    {
      training_padding_ = training_padding;
    }

    bool HOGTrainer::isTrainHardNegetive() const
    {
      return train_hard_negative_;
    }

    void HOGTrainer::setTrainHardNegetive(bool train_hard_negative)
    {
      train_hard_negative_ = train_hard_negative;
    }

    double HOGTrainer::getHitThreshold() const
    {
      return hit_threshold_;
    }

  }
}
