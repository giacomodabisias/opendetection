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
/** \brief Example of the usage of HOG detector
   *
   * \author Kripasindhu Sarkar
   *
   */

#include "od/detectors/global2D/detection/ODHOGDetector.h"
#include "od/common/utils/ODFrameGenerator.h"
#include "od/common/utils/ODUtils.h"
#include <boost/shared_ptr.hpp>

int main(int argc, char *argv[])
{
  std::string trained_data_dir(argv[1]), input_video(argv[2]), output_video = "output.avi";
  if (argc > 3) output_video = argv[3];

  cv::Size size_single(640, 480);

  //get 3 detectors of different types
  std::vector<std::string> messages; 
  messages.push_back("Original");

  std::vector<od::g2d::ODHOGDetector> detectors;
  od::g2d::ODHOGDetector detector1; //
  messages.push_back("OpenCV Default People"); 
  detectors.push_back(detector1);

  od::g2d::ODHOGDetector detector2; 
  detector2.setSvmtype(od::g2d::ODHOGDetector::OD_DAIMLER_PEOPLE);
  messages.push_back("OpenCV Daimler People"); 
  detectors.push_back(detector2);

  od::g2d::ODHOGDetector detector(trained_data_dir);
  messages.push_back("Custom HOG from trained data"); 
  detectors.push_back(detector);

  //init all detectors
  for(size_t i = 0; i < detectors.size(); ++i) 
    detectors[i].init();

  //get scenes
  od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_DEVICE> frameGenerator(input_video);
  //od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_FILE_LIST> frameGenerator(input_video);
  cv::VideoWriter videoWriter(output_video, CV_FOURCC('M','J','P','G'), 25, size_single * 2, true);

  //GUI
  cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
  cv::Mat multi_image;
  while(frameGenerator.isValid() && cv::waitKey(33) != 27)
  {
    boost::shared_ptr<od::ODSceneImage> scene = frameGenerator.getNextFrame();

    std::vector<cv::Mat> images_to_show;
    images_to_show.push_back(scene->getCVImage()); //push the first image

    //detect 3 times
    for(size_t i = 0; i < detectors.size(); i++)
    {
      boost::shared_ptr<od::ODDetections2D> detections =  detectors[i].detectOmni(scene);
      if(detections->size() > 0)
        images_to_show.push_back(detections->renderMetainfo(*scene).getCVImage());
      else 
        images_to_show.push_back(scene->getCVImage());
    }

    multi_image = od::makeCanvasMultiImages(images_to_show, size_single, messages);
    cv::imshow("Overlay", multi_image);
    videoWriter << multi_image;

  }

  return 0;
}