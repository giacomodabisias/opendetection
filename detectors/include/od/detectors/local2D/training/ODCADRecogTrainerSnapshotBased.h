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
//
// Created by sarkar on 17.03.15.
//
#pragma once
#include "od/common/pipeline/ODTrainer.h"
#include "od/common/utils/utils.h"
#include "od/detectors/local2D/ODImageLocalMatching.h"

#include <string>
#include <stdlib.h>

#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkRendererCollection.h>
#include <vtkCommand.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkOBJReader.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCell.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <string>
#include <vtkRendererCollection.h>
#include <vtkCellArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkPlaneSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropPicker.h>
#include <vtkPointPicker.h>

#include <vtkImageReader2Factory.h>
#include <vtkImageReader.h>
#include <vtkTexture.h>
#include <vtkAxesActor.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#include <sstream>
#include <opencv2/core/types.hpp>
#include <map>

#include <boost/filesystem.hpp>

//opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <vtkJPEGWriter.h>

#include "pugixml.hpp"

#define VIEW_ANGLE 30
#define NO_SNAPSHOTS 30

namespace od
{
  namespace l2d
  {
    /** \brief ODCADRecogTrainerSnapshotBased; One of the new algorithm; details will be explained later
   *
   * \author Kripasindhu Sarkar
   *
   */

    class ODCADRecogTrainerSnapshotBased : public ODImageLocalMatchingTrainer
    {

    public:

      ODCADRecogTrainerSnapshotBased(const std::string & training_input_location_ = "", const std::string & training_data_location_ = "") : 
                                     ODImageLocalMatchingTrainer(training_input_location_, training_data_location_){}

      int train();
      void init() {}
      void trainSingleModel(const std::string & objname);

    protected:

      int no_ring_;
      float view_angle_;
      int no_snapshot_per_ring_;


    };
  }
}

