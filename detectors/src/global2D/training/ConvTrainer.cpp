#include "od/detectors/global2D/training/ConvTrainer.h"

namespace od
{
	namespace g2d
	{

		ConvTrainer::ConvTrainer(const std::string & training_input_location, 
						         const std::string & trained_data_location):
								 Trainer(training_input_location, trained_data_location){}

		int ConvTrainer::train()
		{
			return 1;
		}
		
		void ConvTrainer::setSolverLocation(const std::string & location)
		{
			solverLocation = location;	
		}
		
		void ConvTrainer::setSolverProperties(int argc, char *argv[])
		{
			auto app = Gtk::Application::create(argc, argv, std::string("org.gtkmm.example"));
			SolverProperties solverProperties;
			solverProperties.set_default_geometry(10000, 10000);
			app->run(solverProperties);
			solverLocation = solverProperties.solverFileName;
		}
		
		void ConvTrainer::createCustomNetwork(int argc, char *argv[])
		{
			auto app = Gtk::Application::create(argc, argv, std::string("org.gtkmm.example"));
			NetworkCreator networkCreator;
			networkCreator.set_default_geometry(10000, 10000);
			app->run(networkCreator);
		}

		void ConvTrainer::startTraining()
		{
#if(WITH_GPU)
			caffe::Caffe::SetDevice(0);
			caffe::Caffe::set_mode(caffe::Caffe::GPU);
#else
			caffe::Caffe::set_mode(caffe::Caffe::CPU);
#endif
			caffe::SGDSolver<float> s(solverLocation);
			s.Solve();
		}

		
	}
}
