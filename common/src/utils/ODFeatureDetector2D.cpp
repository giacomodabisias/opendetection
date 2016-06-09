//
// Created by sarkar on 20.04.15.
//

#include <GL/gl.h>
#include "od/common/utils/ODFeatureDetector2D.h"

namespace od
{

  ODFeatureDetector2D::ODFeatureDetector2D(const std::string & feature_type, bool use_gpu)
  {
    mode_ = SIFT;

    if(use_gpu) {
      if(feature_type == "ORB") {
        mode_ = ORB_GPU;
        feature_detector_ = cv::cuda::ORB::create();
      }else if(feature_type == "SIFT") {
        mode_ = SIFT_GPU;
        sift_gpu_ = new SiftGPU();
        char *argv[] = {(char *) "-fo", (char *) "-1", (char *) "-v", (char *) "1"};
        int argc = sizeof(argv) / sizeof(char *);
        sift_gpu_->ParseParam(argc, argv);
        if(sift_gpu_->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
          std::cout << "FATAL ERROR cannot create SIFTGPU context" << std::endl;
      }
    } else {
      if(feature_type == "SIFT") {
        mode_ = SIFT;
        feature_detector_ = cv::xfeatures2d::SIFT::create();
      } else if(feature_type == "ORB") {
        mode_ = ORB;
        feature_detector_ = cv::ORB::create();
      } else if(feature_type == "SURF") {
        mode_ = SURF;
        feature_detector_ = cv::xfeatures2d::SURF::create();
      }
    }
  }

  ODFeatureDetector2D::ODFeatureDetector2D(FeatureType type)
  {

    mode_ = type;

    switch(type)
    {
      case(SIFT) : 
        feature_detector_ = cv::xfeatures2d::SIFT::create();
        break;
      
      case(ORB) : 
        feature_detector_ = cv::ORB::create();
        break;
      
      case(SURF) : 
        feature_detector_ = cv::xfeatures2d::SURF::create();
        break;
      
      case(ORB_GPU) : 
        feature_detector_ = cv::cuda::ORB::create();
        break;
      default :
        sift_gpu_ = new SiftGPU();
        char *argv[] = {(char *) "-fo", (char *) "-1", (char *) "-v", (char *) "3", (char *) "-cuda"};
        int argc = sizeof(argv) / sizeof(char *);
        sift_gpu_->ParseParam(argc, argv);
        if(sift_gpu_->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
          std::cout << "FATAL ERROR cannot create SIFTGPU context";
        break;
      
    }
  }

  void ODFeatureDetector2D::computeKeypointsAndDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
  {
    if(mode_ == SIFT_GPU) {
      findSiftGPUDescriptors_(image, descriptors, keypoints);
    } else {
      feature_detector_->detect(image, keypoints);
      feature_detector_->compute(image, keypoints, descriptors);
    }
  }

  void CVMatToSiftGPU(const cv::Mat & image, unsigned char * siftImage, cv::Mat & grey)
  {
    siftImage = (unsigned char *) malloc(image.rows * image.cols);
    cv::Mat tmp;
    cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);

    memcpy(siftImage, grey.data, image.rows * image.cols);
  }

  void ODFeatureDetector2D::findSiftGPUDescriptors_(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
  {
    unsigned char * data = image.data;
    cv::Mat greyimage;
    if(image.type() != CV_8U) {
      cv::cvtColor(image, greyimage, cv::COLOR_BGR2GRAY);
      data = greyimage.data;
    }
    sift_gpu_->RunSIFT(image.cols, image.rows, data, GL_LUMINANCE, GL_UNSIGNED_BYTE);

    int nFeat = sift_gpu_->GetFeatureNum();//get feature count
    //allocate memory for readback
    std::vector<SiftGPU::SiftKeypoint> keys(nFeat);
    //read back keypoints and normalized descritpros
    //specify NULL if you don’t need keypoints or descriptors
    std::vector<float> imageDescriptors(128 * nFeat);
    sift_gpu_->GetFeatureVector(&keys[0], &imageDescriptors[0]);

    sift_gpu_->SaveSIFT("2.sift");

    //to opencv format
    keypoints.clear();
    descriptors.create(0, 128, CV_32FC1);
    for(int i = 0; i < nFeat; ++i) {
      cv::KeyPoint key(keys[i].x, keys[i].y, keys[i].s, keys[i].o);
      keypoints.push_back(key);
      cv::Mat descriptor(1, 128, CV_32FC1);

      for(int x = 0; x < 128; x++)
        descriptor.at<float>(x) = floor(0.5 + (512.0f * imageDescriptors[(i * 128) + x]));

      descriptors.push_back(descriptor);
    }
    //viewImage(image, keypoints);

  }

  void ODFeatureDetector2D::findSiftGPUDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
  {
    unsigned char *data = image.data;
    cv::Mat greyimage;
    if(image.type() != CV_8U) {
      cv::Mat tmp;
      cv::cvtColor(image, tmp, cv::COLOR_BGR2GRAY);
      data = tmp.data;
    }
    sift_gpu_->RunSIFT(image.cols, image.rows, data, GL_LUMINANCE, GL_UNSIGNED_BYTE);

    int nFeat = sift_gpu_->GetFeatureNum();//get feature count
    //allocate memory for readback
    std::vector<SiftGPU::SiftKeypoint> keys(nFeat);
    //read back keypoints and normalized descritpros
    //specify NULL if you don’t need keypoints or descriptors
    std::vector<float> imageDescriptors(128 * nFeat);
    sift_gpu_->GetFeatureVector(&keys[0], &imageDescriptors[0]);

    sift_gpu_->SaveSIFT("2.sift");

    //to opencv format
    keypoints.clear();
    cv::Mat descriptormat = cv::Mat(1, nFeat, CV_32F, &imageDescriptors[0]);
    descriptormat.copyTo(descriptors);

    for(int i = 0; i < nFeat; ++i) {
      cv::KeyPoint key(keys[i].x, keys[i].y, keys[i].s, keys[i].o);
      keypoints.push_back(key);
    }


    //viewImage(image, keypoints);

  }

  void ODFeatureDetector2D::findSiftGPUDescriptors(const std::string & image_name, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
  {
    sift_gpu_->RunSIFT(image_name.c_str());

    int nFeat = sift_gpu_->GetFeatureNum();//get feature count
    //allocate memory for readback
    std::vector<SiftGPU::SiftKeypoint> keys(nFeat);
    //read back keypoints and normalized descritpros
    //specify NULL if you don’t need keypoints or descriptors
    std::vector<float> imageDescriptors(128 * nFeat);
    sift_gpu_->GetFeatureVector(&keys[0], &imageDescriptors[0]);

    sift_gpu_->SaveSIFT("1.sift");

    //to opencv format
    keypoints.clear();
    descriptors.create(0, 128, CV_32FC1);
    for(size_t i = 0; i < nFeat; ++i) {
      cv::KeyPoint key(keys[i].x, keys[i].y, keys[i].s, keys[i].o);
      keypoints.push_back(key);
      cv::Mat descriptor(1, 128, CV_32FC1);
      for(size_t x = 0; x < 128; x++) 
        descriptor.at<float>(x) = floor(0.5 + 512.0f * imageDescriptors[(i << 7) + x]);
      descriptors.push_back(descriptor);
    }
    cv::Mat image = cv::imread(image_name);
  }

  void ODFeatureDetector2D::computeAndSave(const cv::Mat & image, const std::string & path)
  {
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    if(mode_ == SIFT_GPU) {
      findSiftGPUDescriptors_(image, descriptors, keypoints);
      sift_gpu_->SaveSIFT(path.c_str());
    } else {
      //DO NOTHING! IMPLEMENT LATER

    }
  }
}
