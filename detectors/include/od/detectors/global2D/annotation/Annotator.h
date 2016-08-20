#pragma once

#include "od/common/pipeline/Trainer.h"
#include "od/common/utils/Utils.h"
#include <iostream>

#include "Annotation.h"

namespace od
{
	namespace g2d
	{

		class Annotator : public Trainer
		{
			public:

				Annotator(const std::string & training_input_location_ = std::string(""), const std::string & trained_data_location_ = std::string("")):
				            Trainer(training_input_location_, trained_data_location_){}

				int train();
				void init(){}
				void startAnnotator(int argc, char *argv[]);
		};
	}
}

