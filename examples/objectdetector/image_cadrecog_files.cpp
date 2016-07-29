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

#include "od/detectors/local2D/training/CADRecogTrainerSnapshotBased.h"
#include "od/detectors/local2D/detection/CADRecognizer2DLocal.h"
#include "od/common/utils/FrameGenerator.h"
#include "od/common/utils/Viewer.h"
#include <boost/shared_ptr.hpp>

int main(int argc, char *argv[])
{

  if(argc < 4){
    std::cout << "Wrong number of parameters: training_dir, trained_data_dir, query_images_dir" << std::endl;
    return -1;
  }

  std::string training_input_dir(argv[1]);
  std::string trained_data_dir(argv[2]);
  std::string query_images(argv[3]);

  //trainer
  od::l2d::CADRecogTrainerSnapshotBased trainer(training_input_dir, trained_data_dir);
  //trainer->train();

  //detector
  od::l2d::CADRecognizer2DLocal detector(trained_data_dir);
  //set commandline options type inputs
  detector.parseParameterString("--use_gpu --method=1 --error=2 --confidence=0.8 --iterations=500 --inliers=20 --metainfo");
  detector.setCameraIntrinsicFile("/home/sarkar/models/opendetection_local/image_local_scenes/camera_householdnew.xml");   //set some other inputs
  detector.init();

  //get scenes
  od::FrameGenerator<od::SceneImage, od::GENERATOR_TYPE_FILE_LIST> frameGenerator(query_images);
  //GUI
  od::Viewer viewer;
  viewer.initCVWindow(std::string("Overlay"));

  while(frameGenerator.isValid() && viewer.wait(10) != 27)
  {
    boost::shared_ptr<od::SceneImage> scene = frameGenerator.getNextFrame();
    cv::imshow("Overlay", scene->getCVImage());

    //Detect
    boost::shared_ptr<od::Detections3D> detections =  detector.detectOmni(scene);

    if(detections->size() > 0)
      viewer.update(detections->getMetainfoImage(), "Overlay");
    else
      viewer.update(scene, "Overlay");

  }

  return 0;
}