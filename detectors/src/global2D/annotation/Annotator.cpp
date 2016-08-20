#include "od/detectors/global2D/annotation/Annotator.h"

namespace od
{
	namespace g2d
	{
		int Annotator::train()
		{
			return 1;
		}

		void Annotator::startAnnotator(int argc, char *argv[])
		{
			auto app = Gtk::Application::create(argc, argv, std::string("org.gtkmm.example"));
			Annotation annotation;
			annotation.set_default_geometry(10000, 10000);
			app->run(annotation);
		}
	}
}			
