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
#include "od/detectors/local2D/ODImageLocalMatching.h"
#include "pugixml.hpp"

#include <vtkSphereSource.h>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkOBJReader.h>
#include <vtkRendererCollection.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropPicker.h>
#include <vtkJPEGWriter.h>
#include <vtkImageReader2Factory.h>
#include <vtkImageReader.h>
#include <vtkWindowToImageFilter.h>
#include <vtkSmartPointer.h>

#include <opencv2/xfeatures2d.hpp>

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

      ODCADRecogTrainerSnapshotBased(const std::string & training_input_location = std::string(""),
                                     const std::string & training_data_location = std::string("")) : 
                                     ODImageLocalMatchingTrainer(training_input_location, training_data_location){}

      int train();
      void init() {}
      void trainSingleModel(const std::string & objname);

    protected:

      int no_ring_;
      float view_angle_;
      int no_snapshot_per_ring_;

    };

    struct fcomp3d_euclidian
    {
      bool operator()(const cv::Point3f & lhs, const cv::Point3f & rhs) const
      { 
        return lhs.x < rhs.x; 
      }
    };

    class vtkTimerCallbackSnapshot : public vtkCommand
    {
    public:

      static vtkTimerCallbackSnapshot * New()
      {
        vtkTimerCallbackSnapshot * cb = new vtkTimerCallbackSnapshot;
        cb->snap_count_ = 0;
        cb->snap_mode = true;
        cb->feature_type = std::string("SIFT");
        return cb;
      }

      virtual void Execute(vtkObject * caller, unsigned long eventId, void * vtkNotUsed(callData));

    private:

      int snap_count_;

    public:

      std::string takeSnapshot(vtkRenderWindow * render_window, int snap_no);

      void write_pairs(const std::vector<std::pair<cv::Point3f, cv::KeyPoint> > & pairs, const cv::Mat & descriptors, const std::string & file_name);

      void write_pairs_xml(const std::vector<std::pair<cv::Point3f, cv::KeyPoint> > & pairs, const cv::Mat & descriptors, const std::string & file_name);

      void process_image(const std::string & img_name, vtkRenderer * ren, vtkActor * actor, int ino);

      void process(vtkRenderWindowInteractor * iren, vtkActor * actor, vtkRenderer * renderer, int ino);

      //some local variables used

      std::vector<std::pair<cv::Point3f, cv::KeyPoint> > pairs_3d_2d;
      cv::Mat common_descriptors;
      std::map<cv::Point3f, cv::KeyPoint, fcomp3d_euclidian> map_3d_2d;
      std::string feature_type;
      std::string input_file, input_dir, output_dir, output_extension;

      vtkActor * actor;
      vtkRenderer * renderer;
      bool snap_mode;

    };


  }
}

