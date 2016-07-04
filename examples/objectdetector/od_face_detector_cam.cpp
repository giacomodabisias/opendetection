#include "od/detectors/global2D/detection/ODFaceDetector.h"
#include "od/common/utils/ODFrameGenerator.h"
#include "od/common/utils/ODShared_pointers.h"

#include <iostream>
#include <string>

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

 
#ifdef WITH_GPU
	od::g2d::ODFaceDetector2D detector(gpu ? od::g2d::ODFaceDetector2D::GPU : od::g2d::ODFaceDetector2D::CPU, argv[1]);
#else
	od::g2d::ODFaceDetector2D detector(od::g2d::ODFaceDetector2D::CPU, argv[1]);
#endif
	//GUI
	cv::namedWindow("Faces");
	while(frame_generator->isValid() && cv::waitKey(10) != 27)
	{
	  boost::shared_ptr<od::ODSceneImage> scene = frame_generator->getNextFrame();

	  //Detect
	  boost::shared_ptr<od::ODDetections2D> detections = detector.detectOmni(scene);

	  if(detections->size() > 0)
	    cv::imshow("Faces", detections->renderMetainfo(scene).getCVImage());
	  else
	    cv::imshow("Faces", scene->getCVImage());
	}

	return 0;
}