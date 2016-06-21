/*
 * RobustMatcher.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: eriba
 */

#include "od/detectors/local2D/simple_ransac_detection/ODRobustMatcher.h"

namespace od {
  
  namespace l2d {

/*
    void convertToUnsignedSiftGPU(const cv::Mat & cv_des, std::vector<unsigned char> & siftgpu_des)
    {
      siftgpu_des.resize(cv_des.rows * 128);

      int desi = 0;

      for (int i = 0; i < cv_des.rows; i++){
        for (int j = 0; j < 128; j++, desi++)
          siftgpu_des[desi] = (unsigned  char)cv_des.at<float>(i,j);
      }
    }

    void convertToDmatch(int siftgpu_matches[][2], unsigned int num_match, std::vector<cv::DMatch> & cv_matches)
    {
      cv_matches.resize(num_match);
      for(size_t i  = 0; i < num_match; ++i)
      {
        //How to get the feature matches:
        //SiftGPU::SiftKeypoint & key1 = keys1[match_buf[i][0]];
        //SiftGPU::SiftKeypoint & key2 = keys2[match_buf[i][1]];
        //key1 in the first image matches with key2 in the second image
        cv_matches[i].trainIdx = siftgpu_matches[i][0]; //Assigned first at init
        cv_matches[i].queryIdx = siftgpu_matches[i][1];
      }
    }

    void getGoodMatches(const std::vector<std::vector<cv::DMatch> > & matches, std::vector<cv::DMatch> & goodMatches)
    {
      for (std::vector<std::vector<cv::DMatch> >::const_iterator
               matchIterator1 = matches.begin(); matchIterator1 != matches.end(); ++matchIterator1)
      {
        // ignore deleted matches
        if (matchIterator1->empty() || matchIterator1->size() < 2)
          continue;
        if ((*matchIterator1)[0].distance < (*matchIterator1)[1].distance)
          goodMatches.push_back((*matchIterator1)[0]);
      }
    }
*/

    // Set the feature detector
    void RobustMatcher::setFeatureDetector(shared_ptr<cv::FeatureDetector> detect)
    { 
      detector_ = detect; 
    }

    // Set the descriptor extractor
    void RobustMatcher::setDescriptorExtractor(shared_ptr<cv::DescriptorExtractor> desc) 
    { 
      extractor_ = desc; 
    }

    // Set the matcher
    void RobustMatcher::setDescriptorMatcher(shared_ptr<cv::DescriptorMatcher> match) 
    { 
      matcher_ = match; 
    }

    void RobustMatcher::instantiateMatcher(const Model & model, bool use_gpu)
    {

      if (use_gpu_ == true) {

        //####GPU

        //1. for SIFT
        matcher_gpu_ = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2);
        cv::cuda::GpuMat train_descriptors(model.getDescriptors());
        matcher_gpu_->add(std::vector<cv::cuda::GpuMat>(1, train_descriptors));
        std::cout << "GPU matcher instantiated ..." << std::endl;

      }
      else
      {
        matcher_ = make_shared<cv::FlannBasedMatcher>();
      }
    }

    RobustMatcher::RobustMatcher(const Model & model, bool use_gpu, bool use_gpu_match)
    {

      use_gpu_ = use_gpu_match;

      // ORB is the default feature
      ratio_ = 0.8f;
      detector_ = shared_ptr<cv::FeatureDetector>(cv::ORB::create().get());
      extractor_ = shared_ptr<cv::DescriptorExtractor>(cv::ORB::create().get());

      instantiateMatcher(model, use_gpu_match);

    //  // BruteFroce matcher with Norm Hamming is the default matcher
    //  //matcher_ = cv::makePtr<cv::BFMatcher>((int)cv::NORM_HAMMING, false);
    //  //matcher_ = cv::makePtr<cv::BFMatcher>((int)cv::NORM_L2, false);
    //
    //
    //  //####################TEMPORARY CODE BELOW!!!!!!!!!!!!!!!!!!!!!!!!
    //
    //  cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1); // instantiate LSH index parameters
    //  cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);       // instantiate flann search parameters
    //  // instantiate FlannBased matcher
    //  //Ptr<DescriptorMatcher> matcher = makePtr<FlannBasedMatcher>(indexParams, searchParams);
    //  //matcher_ =  cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
    //  matcher_ =  cv::makePtr<cv::FlannBasedMatcher>();

    }

    RobustMatcher::~RobustMatcher()
    {
      // TODO Auto-generated destructor stub
    }

    void RobustMatcher::computeKeyPoints(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints)
    {
      detector_->detect(image, keypoints);
    }

    void RobustMatcher::computeDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors)
    {
      extractor_->compute(image, keypoints, descriptors);
    }

    int RobustMatcher::ratioTest(std::vector<std::vector<cv::DMatch> > & matches)
    {
      int removed = 0;
      // for all matches
      for(auto & match : matches)
      {
        // if 2 NN have been identified
        if(match.size() > 1)
        {
          // check distance ratio
          if(match[0].distance / match[1].distance > ratio_)
          {
            match.clear(); // remove match
            removed++;
          }
        }
        else
        { // does not have 2 neighbours
          match.clear(); // remove match
          removed++;
        }
      }
      return removed;
    }

    void RobustMatcher::setRatio(float rat) 
    { 
      ratio_ = rat; 
    }

    void RobustMatcher::symmetryTest(const std::vector<std::vector<cv::DMatch> > & matches1,
                         const std::vector<std::vector<cv::DMatch> >& matches2,
                         std::vector<cv::DMatch> & symMatches )
    {

      // for all matches image 1 -> image 2
       for(auto & match1 : matches1)
       {
          // ignore deleted matches
          if(match1.empty() || match1.size() < 2)
             continue;

          // for all matches image 2 -> image 1
          for(auto & match2 :  matches2)
          {
            // ignore deleted matches
            if(match2.empty() || match2.size() < 2)
               continue;

            // Match symmetry test
            if(match1[0].queryIdx == match2[0].trainIdx && match2[0].queryIdx == match1[0].trainIdx)
            {
                // add symmetrical match
                symMatches.push_back(cv::DMatch(match1[0].queryIdx, match1[0].trainIdx, match1[0].distance));
                break; // next match in image 1 -> image 2
            }
          }
       }

    }

    void RobustMatcher::match(const cv::Mat & descriptors_frame, const cv::Mat & descriptors_model, std::vector<cv::DMatch> & good_matches)
    {
      std::vector<std::vector<cv::DMatch> > matches;

      if(use_gpu_)
      {
        matcher_gpu_->knnMatch(cv::cuda::GpuMat(descriptors_frame), matches, 2); // return 2 nearest neighbours
      }else{
        matcher_->knnMatch(descriptors_frame, descriptors_model, matches, 2); // return 2 nearest neighbours
      }

      ratioTest(matches);
      // 4. Fill good matches container
      for(auto & match : matches)
      {
        good_matches.push_back(match[0]);
      }

    }

    void RobustMatcher::matchNormalized(cv::Mat & descriptors_frame, cv::Mat & descriptors_model, std::vector<cv::DMatch> & good_matches)
    {
      od::normL2(descriptors_frame); 
      od::normL2(descriptors_model);
      match(descriptors_frame, descriptors_model, good_matches);
    }

    void RobustMatcher::robustMatch(const cv::Mat & frame, std::vector<cv::DMatch> & good_matches,
                                    std::vector<cv::KeyPoint> & keypoints_frame, const cv::Mat & descriptors_model )
    {
      // 1a. Detection of the ORB features
      //this->computeKeyPoints(frame, keypoints_frame);

      // 1b. Extraction of the ORB descriptors
      cv::Mat descriptors_frame;
      //this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

      featureDetector_->computeKeypointsAndDescriptors(frame, descriptors_frame, keypoints_frame);
      //cout << "After Restacking: \n" << descriptors_frame << endl;
      // 2. Match the two image descriptors
      std::vector<std::vector<cv::DMatch> > matches12, matches21;

      // 2a. From image 1 to image 2
      matcher_->knnMatch(descriptors_frame, descriptors_model, matches12, 2); // return 2 nearest neighbours

      // 2b. From image 2 to image 1
      matcher_->knnMatch(descriptors_model, descriptors_frame, matches21, 2); // return 2 nearest neighbours

      // 3. Remove matches for which NN ratio is > than threshold
      // clean image 1 -> image 2 matches
      ratioTest(matches12);
      // clean image 2 -> image 1 matches
      ratioTest(matches21);

      //get_good_matches(matches12, good_matches);

      // 4. Remove non-symmetrical matches
      symmetryTest(matches12, matches21, good_matches);
    }

    void RobustMatcher::findFeatureAndMatch(const cv::Mat & frame, std::vector<cv::DMatch> & good_matches,
                                            std::vector<cv::KeyPoint> & keypoints_frame,
                                            const cv::Mat & descriptors_model )
    {
      good_matches.clear();

      // 1b. Extraction of the ORB descriptors
      cv::Mat descriptors_frame;
      featureDetector_->computeKeypointsAndDescriptors(frame, descriptors_frame, keypoints_frame);

      std::cout << "df " << descriptors_model.rows << std::endl;
      std::cout << "kf" << descriptors_frame.rows << std::endl;

      match(descriptors_frame, descriptors_model, good_matches);

    }


    void RobustMatcher::fastRobustMatch(const cv::Mat & frame, std::vector<cv::DMatch> & good_matches,
                                        std::vector<cv::KeyPoint> & keypoints_frame,
                                        const cv::Mat & descriptors_model )
    {
      good_matches.clear();

      cv::Mat d_f, d_m;
      // 1a. Detection of the ORB features
      //this->computeKeyPoints(frame, keypoints_frame);

      // 1b. Extraction of the ORB descriptors
      cv::Mat descriptors_frame;
      //this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

      featureDetector_->computeKeypointsAndDescriptors(frame, descriptors_frame, keypoints_frame);

      if(descriptors_frame.type() != CV_32F) {
        descriptors_frame.convertTo(d_f, CV_32F);
      }
      else{
        d_f = descriptors_frame;
      }
      if(descriptors_model.type() != CV_32F) {
        descriptors_model.convertTo(d_f, CV_32F);
      }
      else
      {
        d_m = descriptors_model;
      }

      // 2. Match the two image descriptors
      std::vector<std::vector<cv::DMatch> > matches;
      matcher_->knnMatch(d_f, d_m, matches, 2);

      // 3. Remove matches for which NN ratio is > than threshold
      ratioTest(matches);

      // 4. Fill good matches container
      for(auto & match : matches)
      {
        good_matches.push_back(match[0]);
      }
    }

  }

}
