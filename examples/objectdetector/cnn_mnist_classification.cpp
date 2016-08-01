#include "od/detectors/global2D/detection/ConvClassification.h"
#include <fstream>

void help()
{
	std::cout << "Usage: ./examples/objectdetector/od_cnn_mnist_classification <path to weight caffemodel file> <path to network file> <path to image file>" << std::endl;
	std::cout << "Example: ./examples/objectdetector/od_cnn_mnist_classification ../examples/objectdetector/Mnist_Classify/mnist.caffemodel ../examples/objectdetector/Mnist_Classify/lenet.prototxt ../examples/objectdetector/Mnist_Classify/3.png" << std::endl << std::endl;
	exit(-1);
}

int main(int argc, char **argv)
{
	if (argc != 4) 
	{
		help();	
	}

	od::g2d::ConvClassification mnist_classifier("");
	mnist_classifier.setWeightModelFileLocation(argv[1]);
	mnist_classifier.setNetworkModelFileLocation(argv[2]);
	mnist_classifier.setImageFileLocation(argv[3]);

	for(unsigned int i = 1; i < argc; i++)
	{
		std::string check(argv[i]);
		if(i == 1)
		{
			size_t found = check.find(".caffemodel");
			if(found > check.length())
			{
				std::cout << "First argument should be caffe weight caffemodel file" << std::endl;
				help();
			}
		}
		else if(i == 2)
		{
			size_t found = check.find(".prototxt");
			if(found > check.length())
			{
				std::cout << "Second argument should be caffe network prototxt file" << std::endl;
				help();
			}
			
		}
		std::ifstream ifs;
		ifs.open(check.c_str());
		if(!ifs)
		{
			std::cout << "File not found: " << check << std::endl;
			help();
		}
	}
	std::string weight_file_location = mnist_classifier.getWeightModelFileLocation();
	std::string network_file_location = mnist_classifier.getNetworkModelFileLocation();
	std::string image_file_location = mnist_classifier.getImageFileLocation();

	mnist_classifier.setTestBlob(1, 28, 28);
	mnist_classifier.classify();

	return 0;
}


