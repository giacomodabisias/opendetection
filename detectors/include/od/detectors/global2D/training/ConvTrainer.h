#pragma once
#include "od/common/pipeline/Trainer.h"
#include "od/common/utils/Utils.h"
#include "od/detectors/global2D/training/Solver.h"
#include "od/detectors/global2D/training/Network.h"
#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <caffe/caffe.hpp>
#include <caffe/util/io.hpp>
#include <caffe/blob.hpp>
#include <caffe/solver.hpp>
#include <caffe/sgd_solvers.hpp>

namespace od
{
	namespace g2d
	{
		class ConvTrainer : public Trainer
		{
			public:
				ConvTrainer(const std::string & training_input_location_ = std::string(""), 
					        const std::string & trained_data_location_ = std::string(""));
				int train();
				void init(){}
				void setSolverLocation(const std::string & location);
				void setSolverProperties(int argc, char *argv[]);
				void startTraining();
				void createCustomNetwork(int argc, char *argv[]);

			private:
				std::string solverLocation;
		};

	}
}

