#include "od/detectors/global2D/detection/ConvClassification.h"

namespace od
{
	namespace g2d
	{

		ConvClassification::ConvClassification(const std::string & trained_data_location):
			Detector2D(trained_data_location){}
		
		void ConvClassification::setWeightModelFileLocation(const std::string & location)
		{
			weightModelFileLoaction = location;
		}

		void ConvClassification::setNetworkModelFileLocation(const std::string & location)
		{
			networkFileLocation = location;
		}

		void ConvClassification::setImageFileLocation(const std::string & location)
		{
			imageFileLocation = location;
		}	
		
		std::string ConvClassification::getWeightModelFileLocation()
		{
			std::cout << "Weight Model File Location = " << weightModelFileLoaction << std::endl;
			return weightModelFileLoaction;
		}
		
		std::string ConvClassification::getNetworkModelFileLocation()
		{
			std::cout << "Network Model File Location = " << networkFileLocation << std::endl;
			return networkFileLocation;
		}
		
		std::string ConvClassification::getImageFileLocation()
		{
			std::cout << "Image File Location = " << imageFileLocation << std::endl;
			return imageFileLocation;
		}

		void ConvClassification::setTestBlob(int numChannels, int imgHeight, int imgWidth)
		{
			if (!ReadImageToDatum(imageFileLocation, numChannels, imgHeight, imgWidth, &strucBlob)) 
			{
				std::cout << "Image File Not Found" << std::endl;
				exit(-1);
			}
			caffe::Blob<float>* dataBlob = new caffe::Blob<float>(1, strucBlob.channels(), strucBlob.height(), strucBlob.width());
			
			protoBlob.set_num(1);
			protoBlob.set_channels(strucBlob.channels());
			protoBlob.set_height(strucBlob.height());
			protoBlob.set_width(strucBlob.width());
			const int data_size = strucBlob.channels() * strucBlob.height() * strucBlob.width();
			int sizeStrucBlob = std::max<int>(strucBlob.data().size(), strucBlob.float_data_size());
			for(int i = 0; i < sizeStrucBlob; ++i) 
			{
				protoBlob.add_data(0.);
			}
			const std::string & data = strucBlob.data();
			if(data.size() != 0) 
			{
				for (int i = 0; i < sizeStrucBlob; ++i)
				{
					protoBlob.set_data(i, protoBlob.data(i) + (uint8_t)data[i]);
				}
			}

			dataBlob->FromProto(protoBlob);
			inputBlob.push_back(dataBlob);
			
		}

		void ConvClassification::classify()
		{
#if(WITH_GPU)
			caffe::Caffe::SetDevice(0);
			caffe::Caffe::set_mode(caffe::Caffe::GPU);
#else
			caffe::Caffe::set_mode(caffe::Caffe::CPU);
#endif
			caffe::Net<float> net(networkFileLocation, caffe::TEST);
			net.CopyTrainedLayersFrom(weightModelFileLoaction); 
			
			float type = 0.0;
			const std::vector<caffe::Blob<float>*>& result =  net.Forward(inputBlob, &type);
			float max = 0;
			float max_i = 0;
			for(int i = 0; i < 10; ++i)
			{
				float value = result[0]->cpu_data()[i];
				if (max < value)
				{
				max = value;
				max_i = i;
				}
			}
			std::cout << std::endl << std::endl << "****** OUTPUT *******" << std::endl;
			std::cout << "classified image is digit " << max_i << std::endl << std::endl;
		}

		void ConvClassification::init()
		{
		}

		shared_ptr<Detections2D> ConvClassification::detectOmni(shared_ptr<SceneImage> scene)
		{
			return make_shared<Detections2D>();
		}

		shared_ptr<Detections> ConvClassification::detect(shared_ptr<SceneImage> scene)
		{
			return make_shared<Detections>();		
		}
		
	
						
	}
}
