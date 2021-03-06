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

#include "od/detectors/misc/detection/DetectorMultiAlgo.hpp"
#include "od/common/utils/FrameGenerator.h"
#include "od/common/utils/Viewer.h"
#include <boost/shared_ptr.hpp>

int main(int argc, char *argv[])
{

  if(argc < 3){
    std::cout << "Wrong number of parameters: training_dir, query_images" << std::endl;
    return -1;
  }

  std::string trained_data_dir(argv[1]);
  std::string query_images(argv[2]);

  //detector
  od::DetectorMultiAlgo2D<pcl::PointXYZRGBA> detector(trained_data_dir);
  detector.init();

  //get scenes
  od::FrameGenerator<od::SceneImage, od::GENERATOR_TYPE_FILE_LIST> frameGenerator(query_images);
  
  od::Viewer viewer;
  viewer.initCVWindow(std::string("Overlay"));

  while(frameGenerator.isValid() && viewer.wait(10) != 27)
  {
    boost::shared_ptr<od::SceneImage> scene = frameGenerator.getNextFrame();

    //Detect
    boost::shared_ptr<od::Detections2D> detections = detector.detectOmni(scene);

    if(detections->size() > 0)
      viewer.render(detections->renderMetainfo(*scene), "Overlay");
    else
      viewer.render(scene, "Overlay");

  }

  return 0;
}