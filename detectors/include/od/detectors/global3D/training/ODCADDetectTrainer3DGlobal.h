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
#pragma once
#include "od/common/pipeline/ODTrainer.h"
#include <iostream>

namespace od
{
  namespace g3d
  {
/** \brief Training class for the detector ODCADDetector3DGlobal.
   *
   * This class uses PCL 3d_recognition_framework in the background for the training of 3D CAD models (in PLY format) and should be used with ODCADDetector3DGlobal for their detection in a pointcloud.
   * In the training_input_location_ the CAD models should be arranged in the following structure:
   *
   * - \<training_input_location_\>
        - <Category 1 dir>
          - ...
          - ...
        - <Category 2 dir>
          - ...
          - ...
        - ...

    For example:

    - /home/user/DB
      - /home/user/DB/bottles
        - /home/user/DB/bottles/milk.ply
        - /home/user/DB/bottles/wine.ply
      - /home/user/DB/fruit
        - /home/user/DB/fruit/apple.ply
        - /home/user/DB/fruit/banana.ply


    After the training use the detector class ODCADDetector3DGlobal.
   *
   * \author Kripasindhu Sarkar
   *
   */
    class ODCADDetectTrainer3DGlobal : public ODTrainer
    {

    public:
      ODCADDetectTrainer3DGlobal(const std::string & training_input_location_ = "", const std::string & training_data_location_ = "");

      int train();

      void init() {};

      const std::string & getDescName() const;

      void setDescName(const std::string & desc_name);

    protected:
      std::string desc_name_;
    };
  }
}

