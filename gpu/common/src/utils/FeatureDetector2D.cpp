//
// Created by sarkar on 20.04.15.
//

#include "od/gpu/common/utils/FeatureDetector2D.h"

namespace od
{

  namespace gpu
  {


    FeatureDetector2D::FeatureDetector2D(FeatureType type)
    {

      mode_ = type;

      switch(type)
      {

        case(SIFT_GPU) : 
        {
          sift_gpu_ = new SiftGPU();
          char *argv[] = {(char *) "-fo", (char *) "-1", (char *) "-v", (char *) "3", (char *) "-cuda"};
          int argc = sizeof(argv) / sizeof(char *);
          sift_gpu_->ParseParam(argc, argv);
          if(sift_gpu_->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
            std::cout << "FATAL ERROR cannot create SIFTGPU context" << std::endl;
            exit(-1);
          break;
        }
        
        case(ORB_GPU) : 
          feature_detector_ = cv::cuda::ORB::create();
          break;

        case(SURF) :
        case(SIFT) :
        case(ORB) :
        default :
          std::cout << "FATAL ERROR, type is not implemented." << std::endl;
          break;
        
      }
    }

    void FeatureDetector2D::computeKeypointsAndDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
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

    void FeatureDetector2D::findSiftGPUDescriptors_(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
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

    void FeatureDetector2D::findSiftGPUDescriptors(const cv::Mat & image, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
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

    void FeatureDetector2D::findSiftGPUDescriptors(const std::string & image_name, cv::Mat & descriptors, std::vector<cv::KeyPoint> & keypoints)
    {
      sift_gpu_->RunSIFT(image_name.c_str());

      unsigned int nFeat = sift_gpu_->GetFeatureNum();//get feature count
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

    void FeatureDetector2D::computeAndSave(const cv::Mat & image, const std::string & path)
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
  
}
