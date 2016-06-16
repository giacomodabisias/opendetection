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
*///
// Created by sarkar on 16.06.15.
//

#include "od/detectors/global3D/training/ODCADDetectTrainer3DGlobal.h"

namespace od {

  namespace g3d {

    ODCADDetectTrainer3DGlobal::ODCADDetectTrainer3DGlobal(const std::string & training_input_location_, const std::string & training_data_location_): 
                                                           ODTrainer(training_input_location_, training_data_location_)
    {
      desc_name_ = std::string("esf");
      trained_location_identifier_ = std::string("GLOBAL3DVFH");
    }


    int ODCADDetectTrainer3DGlobal::train()
    {
      shared_ptr<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > mesh_source(new pcl::rec_3d_framework::MeshSource<pcl::PointXYZ>());

      mesh_source->setPath(training_input_location_);
      mesh_source->setResolution(150);
      mesh_source->setTesselationLevel(1);
      mesh_source->setViewAngle(57.f);
      mesh_source->setRadiusSphere(1.5f);
      mesh_source->setModelScale(1.f);
      std::string location = getSpecificTrainingDataLocation();
      mesh_source->generate(location);
      
      return 1;
    }

    const std::string & ODCADDetectTrainer3DGlobal::getDescName() const
    {
      return desc_name_;
    }

    void ODCADDetectTrainer3DGlobal::setDescName(const std::string & desc_name)
    {
      desc_name_ = desc_name;
    }

  }
}


