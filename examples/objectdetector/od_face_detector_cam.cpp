#include "od/detectors/global2D/detection/ODCascadeDetector.h"

#include "od/common/utils/ODFrameGenerator.h"
#include "od/common/utils/ODViewer.h"

#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>


int main(int argc, char * argv[])
{

	if(argc < 2){
		std::cout << "Use with parameters: detector_file_path gpu_usage (0 disable / 1 enable)" << std::endl;
		return -1;
	}

	unsigned int gpu = 0;
	shared_ptr<od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_DEVICE> > frame_generator;

	if(argc > 2){
		gpu = atoi(argv[2]);
	}

	frame_generator = make_shared<od::ODFrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_DEVICE> >(0);

	od::ODViewer viewer;
	viewer.initCVWindow(std::string("Faces"));
	
	boost::shared_ptr<od::ODDetector2D> detector;

	//ADD GPU
	detector = boost::make_shared<od::g2d::ODCascadeDetector>(gpu, argv[1]);
	
	detector->init();

	while(frame_generator->isValid() && viewer.wait(100) != 27)
	{
	  boost::shared_ptr<od::ODSceneImage> scene = frame_generator->getNextFrame();

	  boost::shared_ptr<od::ODDetections2D> detections = detector->detectOmni(scene);

	  if(detections->size() > 0)
	  	viewer.update(detections->renderMetainfo(scene), std::string("Faces"));
	  else
	    viewer.update(scene, std::string("Faces"));

		viewer.spin();
	}

	return 0;
}