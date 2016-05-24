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
#include "od/detectors/local2D/training/ODCADRecogTrainerSnapshotBased.h"
#include "od/detectors/local2D/detection/ODCADRecognizer2DLocal.h"
#include "od/common/utils/ODFrameGenerator.h"

int main(int argc, char *argv[])
{

  if(argc < 5){
    std::cout << "Wrong number of parameters: image_path  model_path  camera_path  output_file" << std::endl;
    return -1;
  }

  std::string imagespath(argv[1]);
  std::string modelsPath(argv[2]);
  std::string camerapath(argv[3]);
  std::string outputfile(argv[4]);
  std::ofstream logfile(outputfile.c_str());

  //detector
  od::l2d::ODCADRecognizer2DLocal detector(modelsPath);
  //set commandline options type inputs
  detector.parseParameterString("--use_gpu --method=1 --error=2 --confidence=0.5 --iterations=1000 --inliers=6 --metainfo");
  detector.setCameraIntrinsicFile(camerapath);   //set some other inputs
  detector.init();

  //get scenes
  od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_FILE_LIST> frameGenerator(imagespath);
  //GUI
  //cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
  while(frameGenerator.isValid() && cv::waitKey(1000) != 27)
  {
    od::ODSceneImage * scene = frameGenerator.getNextFrame();
    //cv::imshow("Overlay", scene->getCVImage());

    //Detect
    od::ODDetections3D *detections =  detector.detectOmni(scene);

    if(detections->size() > 0)
    {
      logfile << scene->getPath() << endl;
      logfile << detections->size() << endl;

      for (int i = 0; i < detections->size(); i++)
      {
        od::ODDetection3D *detection = detections->at(i);
        detection->printSelf();
        logfile << detection->getId() << endl;
      }

      //cv::imshow("Overlay", detections->getMetainfoImage());
    }
    else
      //cv::imshow("Overlay", scene->getCVImage());



    delete scene;
    delete detections;

  }

  return 0;
}