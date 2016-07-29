#include "od/detectors/global2D/detection/CascadeDetector.h"

#include "od/common/utils/FrameGenerator.h"
#include "od/common/utils/Viewer.h"

#include <iostream>
#include <boost/shared_ptr.hpp>


int main(int argc, char * argv[])
{

	if(argc < 3){
		std::cout << "Wrong number of parameters: image_dir_path detector_file gpu_usage (0 disable / 1 enable)" << std::endl;
		return -1;
	}

  	std::string image_path(argv[1]);
	od::FrameGenerator<od::SceneImage, od::GENERATOR_TYPE_FILE_LIST> frame_generator(image_path);

	unsigned int gpu = 0;
	if(argc > 3)
		gpu = atoi(argv[3]);

	boost::shared_ptr<od::Detector2D> detector;


	detector = boost::make_shared<od::g2d::CascadeDetector>(gpu, argv[1]);

	
	detector->init();

	od::Viewer viewer;
	viewer.initCVWindow(std::string("Faces"));

	while(frame_generator.isValid() && viewer.wait(10) != 27)
	{
		boost::shared_ptr<od::SceneImage> scene = frame_generator.getNextFrame();

		boost::shared_ptr<od::Detections2D> detections = detector->detectOmni(scene);

		if(detections->size() > 0)
	  		viewer.update(detections->renderMetainfo(scene), std::string("Faces"));
	  	else
	    	viewer.update(scene, std::string("Faces"));

	    viewer.spin();
	    viewer.wait(0);

	}

	return 0;
}