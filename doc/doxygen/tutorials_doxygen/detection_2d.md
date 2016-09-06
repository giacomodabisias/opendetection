Detection 2D {#detection_2d}
====
[TOC]

Detection 2D {#detection_2d1}
===

This article goes through the 2D detection methods covered in OD. Specifically, it covers the classes - od::g2d::HOGDetector through a tutorial.

2D detection methods are performed by the classes Detector2D. They accept a `SceneImage` and performs detection/recognition on them. Currently Detector2Ds are classified into g2d and l2d namespaces. g2d covers detection methods which uses global 2D features (like HOG/Cascade) while l2d covers detection methods which uses local 2D features (like SIFT/SURF/ORB) for detection/recognition. Different 2D detectors that are available currently: od::g2d::HOGDetector, od::g2d::CascadeDetector, od::g2d::FaceRecognizer, od::l2d::CADRecognizer2DLocal.
The gpu module contains the gpu implementation of the algorithms where present. The ::gpu:: namespace has to be added after od in order to use that interface which has usually exactly the same API as the cpu version.

##HOG feature based detection {#detection_2d2}

HOGDetector is a HOG feature based linear classifier. It accepts an image (od::ODSceneImage), computes its HOG feature (in a multiscale mannar for detectOmni() and a single descriptor from the resized image for detect()), runs a linear SVM obtained either by HOGTrainer through training or some default ones (from OpenCV), and informs if the classifier is true thereby providing detection.

A complete example including training is provided in `examples/objectdetector/hog_train.cpp`. For positive and negetive example get the data from the INRIA human dataset: http://pascal.inrialpes.fr/data/human/.
 
In this tutorial we will go through a more complete application in present in `examples/apps/global2D/multihog_app.cpp`. The code is provided here verbatime: 

\code{.cpp}

*
* \author Kripasindhu Sarkar
*
*/

//include all necessary files
#include "od/detectors/global2D/training/HOGTrainer.h"
#include "od/detectors/global2D/detection/HOGDetector.h"
#include "od/common/utils/FrameGenerator.h"
#include "od/common/utils/Viewer.h"
#include <boost/shared_ptr.hpp>

int main(int argc, char *argv[])
{

//Check the input arguments
  if(argc < 5){
    std::cout << "Wrong number of parameters: positive_samples_dir, negative_Samples_dir, trained_data_dir, test_iamges_dir" << std::endl;
    return -1;
  }

  //define all the directories
  std::string pos_samples(argv[1]);
  std::string neg_samples(argv[2]);
  std::string trained_data_dir(argv[3]);
  std::string test_images(argv[4]);

  od::g2d::HOGTrainer trainer("", trained_data_dir); // create the trainer
  trainer.setPosSamplesDir(pos_samples);
  trainer.setNegSamplesDir(neg_samples);
  trainer.setNOFeaturesNeg(10);
  trainer.setTrainHardNegetive(true);
  trainer.train();

  od::g2d::HOGDetector detector; // create the dector
  detector.setTrainedDataLocation(trained_data_dir); //set trained director locations
  detector.init(); //initialize detector

  //get scenes
  od::FrameGenerator<od::SceneImage, od::GENERATOR_TYPE_FILE_LIST> frameGenerator(test_images);
  //GUI
  od::Viewer viewer;
  viewer.initCVWindow(std::string("Overlay"));

  while(frameGenerator.isValid() && viewer.wait(10) != 27)
  {
    boost::shared_ptr<od::SceneImage> scene = frameGenerator.getNextFrame(); //read a frame (image)

    //Detect
    boost::shared_ptr<od::Detections2D> detections = detector.detectOmni(scene);

    if(detections->size() > 0)
      viewer.render(detections->renderMetainfo(*scene), "Overlay");
    else
      viewer.render(scene, "Overlay");

    viewer.spin();
  }

  return 0;
}
\endcode

###Data {#detection_2d3}
This app compares the results of HOG based detection with three different trained classifiers. For this app you need a pre-trained hog descriptor in your `trained_data` directory. You can either train  as in `examples/objectdetector/hog_train.cpp`, or get the OD pre-trained data from the \ref getting_started2 "Data Repository".

For the query video, get a clip containing many pedestrians. For example you can get one from http://www.robots.ox.ac.uk/ActiveVision/Research/Projects/2009bbenfold_headpose/project.html#datasets

Run the app from the build directory as:

    examples/apps/od_multihog_app <path_to_data>/trained_data/ <input_pedestrian_video> <output_comparison_video_with_detection>

Depending on the input video, you will see something like the following:

\htmlonly
<div align="center">
<iframe width="800" height="600" src="https://www.youtube.com/embed/NaED6B-S4ks" frameborder="0" allowfullscreen></iframe>
</div>
\endhtmlonly


###Code explanation {#detection_2d4}
We first init 3 different instances of HOGDetector of different settings.  

  std::vector<od::g2d::HOGDetector> detectors;
  od::g2d::HOGDetector detector1; //
  messages.push_back("OpenCV Default People"); 
  detectors.push_back(detector1);

  od::g2d::HOGDetector detector2; 
  detector2.setSvmtype(od::g2d::HOGDetector::_DAIMLER_PEOPLE);
  messages.push_back("OpenCV Daimler People"); 
  detectors.push_back(detector2);

  od::g2d::HOGDetector detector3(trained_data_dir);
  messages.push_back("Custom HOG from trained data"); 
  detectors.push_back(detector3);      
      
You can set different types of linear SVMs for HOG detector using `setSvmtype` function. For adding a custom SVM use the type CUSTOM and set the linear SVM weight vector using `setSVMDetector()`. You have to then update the other HOG detector parameters accordingly (like winSize etc) with which the your SVM was trained.  If you give a trained data directory, it will use the xml file from the directory. Here, we set three different available HOG detectors.
  
After setting all the parameters you need to call `init()` which is done in the following line.
  
     //init all detectors
     for (int i = 0; i < detectors.size(); i++) 
      detectors[i].init();
     
Then we create a FrameGenerator object, which is a templated class for grabbing both Images and Point Clouds (from kinect). Use od::GENERATOR_TYPE_DEVICE for grabbing frames from camera (kinekt for Point Cloud) or video which is done in the following line.      
      
      //get scenes
      od::FrameGenerator<od::ODSceneImage, od::GENERATOR_TYPE_DEVICE> frameGenerator(input_video); 
      
The next valid frame can be accessed now by `frameGenerator.getNextFrame();` which is being done in loop until exhaustion.   

Each ImageScene is then checked by the detector using the detectOmni() function which searches the whole image for detections. The resultant Detections2D contains information about the detections made (like Type/Bounding box etc). We use an function `renderMetainfo` to draw the bounding box of all the detections made.
 
    //detect 3 times
    for(size_t i = 0; i < detectors.size(); i++)
    {
      boost::shared_ptr<od::Detections2D> detections =  detectors[i].detectOmni(scene);
      if(detections->size() > 0)
        images_to_show.push_back(detections->renderMetainfo(*scene).getCVImage());
      else 
        images_to_show.push_back(scene->getCVImage());
    }

The function `od::makeCanvasMultiImages` take N images and concatenate them in order forming a big image with messages. That concatenated image with having information from all the detectors are then being shown. 
  
    multi_image = od::makeCanvasMultiImages(images_to_show, size_single, messages);
    cv::imshow("Overlay", multi_image);
    
    
