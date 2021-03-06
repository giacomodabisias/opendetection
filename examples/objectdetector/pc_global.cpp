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

/** \brief Example of the usage of global 3D pipeline
   *
   * \author Kripasindhu Sarkar
   *
   */

#include "od/detectors/global3D/PointCloudGlobalMatching.h"
#include <boost/shared_ptr.hpp>
#include <iostream>

int main(int argc, char *argv[])
{

  if(argc < 4){
    std::cout << "Wrong number of parameters: training_dir, trained_data_dir, pointcloud_file" << std::endl;
    return -1;
  }

  std::string training_input_dir(argv[1]);
  std::string trained_data_dir(argv[2]);
  std::string pointcloud_file(argv[3]);

  //trainer
  od::g3d::CADDetectTrainer3DGlobal trainer(training_input_dir, trained_data_dir);
  trainer.train();

  //detector
  od::g3d::CADDetector3DGlobal<pcl::PointXYZRGBA> detector ;
  detector.setTrainingInputLocation(training_input_dir);
  detector.setTrainedDataLocation(trained_data_dir);
  detector.init();

  //Get a scene
  boost::shared_ptr<od::ScenePointCloud<pcl::PointXYZRGBA> > scene = boost::make_shared<od::ScenePointCloud<pcl::PointXYZRGBA>>(pointcloud_file);
  
  boost::shared_ptr<od::Detections3D> detections = detector.detectOmni(scene);

  //feedback
  for(size_t i = 0; i < detections->size(); ++i)
  {
    detections->at(i)->printSelf();
  }

  return 0;
}
