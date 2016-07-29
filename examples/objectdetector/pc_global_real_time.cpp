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
// Created by sarkar on 16.06.15.
//

/** \brief Example of the usage of global pipeline
   *
   * \author Kripasindhu Sarkar
   *
   */


#include "od/detectors/global3D/PointCloudGlobalMatching.h"
#include "od/common/utils/FrameGenerator.h"
#include "od/common/utils/Viewer.h"
#include <string>
#include <boost/shared_ptr.hpp>

int main(int argc, char *argv[])
{

  if(argc < 3){
    std::cout << "Wrong number of parameters: training_dir, trained_data_dir" << std::endl;
    return -1;
  }

  std::string training_input_dir(argv[1]);
  std::string trained_data_dir(argv[2]);

  //trainer
  od::g3d::CADDetectTrainer3DGlobal trainer(training_input_dir, trained_data_dir);
  trainer.train();

  //detector
  od::g3d::CADDetector3DGlobal<pcl::PointXYZRGBA> detector;;
  detector.setTrainingInputLocation(training_input_dir);
  detector.setTrainedDataLocation(trained_data_dir);
  detector.init();

  //GUI and feedback

  od::Viewer viewer;
  viewer.initPCLWindow(std::string("kinect"));
  od::FrameGenerator<od::ScenePointCloud<pcl::PointXYZRGBA>, od::GENERATOR_TYPE_DEVICE> frameGenerator;
  pcl::PointXYZ pos;

  while(frameGenerator.isValid())
  {

    boost::shared_ptr<od::ScenePointCloud<pcl::PointXYZRGBA>> frame = frameGenerator.getNextFrame();

    //remove previous point clouds and text and add new ones in the visualizer
    viewer.removeAll();
    viewer.removeAllShapes();
    viewer.render(frame->getPointCloud(), std::string("frame"));

    //Detect
    boost::shared_ptr<od::Detections3D> detections = detector.detectOmni(frame);

    //add all the detections in the visualizer with its id as text
    for(size_t i = 0; i < detections->size(); ++i)
    {
      pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> random_handler (detections->at(i)->getMetainfoCluster());
      viewer.render(detections->at(i)->getMetainfoCluster(), random_handler, std::string("cluster_") + std::to_string(i));
       
      pos.x = detections->at(i)->getLocation()[0]; 
      pos.y = detections->at(i)->getLocation()[1]; 
      pos.z = detections->at(i)->getLocation()[2];
      viewer.addText(detections->at(i)->getId(), pos, 0.015f, cv::Scalar(1, 0, 1));
    }

    viewer.spin();
  }

  return 0;
}
