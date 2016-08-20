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

		std::vector<float> ConvClassification::classifyMultiLabel()
		{
#if(WITH_GPU)
			caffe::Caffe::SetDevice(0);
			caffe::Caffe::set_mode(caffe::Caffe::GPU);
#else
			caffe::Caffe::set_mode(caffe::Caffe::CPU);
#endif
			caffe::Net<float>  net(networkFileLocation, caffe::TEST);
			net.CopyTrainedLayersFrom(weightModelFileLoaction); 
			
			float type = 0.0;
			const std::vector<caffe::Blob<float>*> & result =  net.Forward(inputBlob, &type);
			std::cout << std::endl << "****** OUTPUT *******" << std::endl;
/*			cout << "The AAM points are " << endl;
			cout << net.output_blobs()[0] << endl;
			for (int i = 0; i < 15; i++)
			{
				cout << result[0]->cpu_data()[2*i]*255 << " " << result[0]->cpu_data()[2*i+1]*255 << " ";
			}
			cout << endl;
*/
			caffe::Blob<float> * output_layer = net.output_blobs()[0];
			const float * begin = output_layer->cpu_data();
			const float * end = begin + output_layer->channels();
			return std::vector<float>(begin, end);
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

		shared_ptr<Detections> ConvClassification::detectOmni(shared_ptr<Scene> scene)
		{
			std::cout << "not implemented, use with shared_ptr<SceneImage>" <<std::endl; 
			return nullptr;
		}

		std::vector<float> ConvClassification::runMultiClassClassifier()
		{
#if(WITH_GPU)
			caffe::Caffe::SetDevice(0);
			caffe::Caffe::set_mode(caffe::Caffe::GPU);
#else
			caffe::Caffe::set_mode(caffe::Caffe::CPU);
#endif
			/* Load the network. */
			caffe::Net<float>  net_m(networkFileLocation, caffe::TEST);
			net_m.CopyTrainedLayersFrom(weightModelFileLoaction);
			caffe::Blob<float>* input_layer = net_m.input_blobs()[0];
			num_channels_ = input_layer->channels();
			input_geometry_ = cv::Size(input_layer->width(), input_layer->height());
			input_layer->Reshape(1, num_channels_,
				       input_geometry_.height, input_geometry_.width);
			net_m.Reshape();
			std::vector<cv::Mat> input_channels;

			int width = input_layer->width();
			int height = input_layer->height();
			float* input_data = input_layer->mutable_cpu_data();
			for (unsigned int i = 0; i < input_layer->channels(); ++i) 
			{
				cv::Mat channel(height, width, CV_32FC1, input_data);
				input_channels.push_back(channel);
				input_data += width * height;
			}


//			net_m.Forward();
			caffe::Blob<float>* output_layer = net_m.output_blobs()[0];
			const float * begin = output_layer->cpu_data();
			const float * end = begin + output_layer->channels();
			return std::vector<float>(begin, end);
		}

		int ConvClassification::runMultiClassClassifierPythonMode()
		{
			std::string mode = "";
#if(WITH_GPU)
			mode = "gpu";
#else
			mode = "cpu";
#endif
			std::string cmd = "python ../examples/objectdetector/AAM_Classify/classify.py " + networkFileLocation + " " + weightModelFileLoaction + " " + imageFileLocation + " " + outputFileLocation + " " + mode;
			return system(cmd.c_str());

		}	

		void ConvClassification::setSegnetLocation(const std::string & location)
		{
			segnetLocation = location;
		}
		void ConvClassification::setImageGroundTruthFileLocation(const std::string & location)
		{
			imageGroundTruthFileLocation = location;
		}
		void ConvClassification::setColorLocation(const std::string & location)
		{
			colorLocation = location;
		}
		int ConvClassification::runSegnetBasedClassifierPythonMode()
		{
			std::string mode = "";
#if(WITH_GPU)
			mode = "gpu";
#else
			mode = "cpu";
#endif
//			string segnet_location = argv[1];
//			string model_file   = argv[2];
//			string trained_file = argv[3];
//			string test_image = argv[4];
//			string test_image_gt = argv[5];
//			string color = argv[6];
//			string mode = argv[7];
			std::string cmd = "python ../examples/objectdetector/Segnet_Classify/test.py " + segnetLocation + " " + networkFileLocation + " " + weightModelFileLoaction + " " + imageFileLocation + " " + imageGroundTruthFileLocation + " " + colorLocation + " " + mode;
			return system(cmd.c_str());
		}	
/*
		std::vector<float> ConvClassification::Predict(const cv::Mat& img)
		{
			Blob<float>* input_layer = net_m.input_blobs()[0];
			input_layer->Reshape(1, num_channels_,
				       input_geometry_.height, input_geometry_.width);
			net_m.Reshape();

			std::vector<cv::Mat> input_channels;
			WrapInputLayer(&input_channels);
			net_m.Forward();
			Blob<float>* output_layer = net_m.output_blobs()[0];
			const float* begin = output_layer->cpu_data();
			const float* end = begin + output_layer->channels();
			return std::vector<float>(begin, end);
		}

		void ConvClassification::WrapInputLayer(std::vector<cv::Mat>* input_channels) 
		{
			Blob<float>* input_layer = net_m.input_blobs()[0];

			int width = input_layer->width();
			int height = input_layer->height();
			float* input_data = input_layer->mutable_cpu_data();
			for (int i = 0; i < input_layer->channels(); ++i) 
			{
				cv::Mat channel(height, width, CV_32FC1, input_data);
				input_channels->push_back(channel);
				input_data += width * height;
			}
		}
*/		
		
	
						
	}
}
