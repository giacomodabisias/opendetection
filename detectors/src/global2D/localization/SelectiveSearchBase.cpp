#include "od/detectors/global2D/localization/SelectiveSearchBase.h"

namespace od
{
	namespace g2d
	{

		SelectiveSearchBase::SelectiveSearchBase(const std::string & trained_data_location):
		                     Detector2D(trained_data_location){}

		void SelectiveSearchBase::acquireImages(const std::string & imageLocation, int imgWidth, int imgHeight)
		{
			inputImageHeight = imgHeight;
			inputImageWidth = imgWidth;
			img = cv::imread(imageLocation, CV_LOAD_IMAGE_COLOR);
			cv::Size size(imgWidth, imgHeight);
			cv::resize(img, img, size);
			cluster = cv::imread(imageLocation, CV_LOAD_IMAGE_COLOR);
			cv::resize(cluster, cluster, size);
			outputImg = cv::imread(imageLocation, CV_LOAD_IMAGE_COLOR);
			cv::resize(outputImg, outputImg, size);
		}


		cv::Mat SelectiveSearchBase::preProcessImg(const cv::Mat & image)
		{
			std::vector<cv::Mat> channels; 
			cv::Mat img_hist_equalized;
			cv::cvtColor(image, img_hist_equalized, CV_BGR2YCrCb); 
			cv::split(img_hist_equalized,channels); 
			cv::equalizeHist(channels[0], channels[0]); 
			cv::merge(channels,img_hist_equalized);
			cv::cvtColor(img_hist_equalized, img_hist_equalized, CV_YCrCb2BGR);
			return img_hist_equalized;
		}

		std::vector<std::vector<int> > SelectiveSearchBase::getSuperPixels(const cv::Mat & im, int & totalMasks, float Sigma, float K, 
			                                                               float Min_size, const std::string & imageLocation)
		{
			std::string img_location(imageLocation + std::string("img.ppm"));
			imwrite(img_location, im);
			float sigma = Sigma;
			float k = K;
			int min_size = Min_size;
			image<rgb> *input = loadPPM(img_location.c_str());
			int num_ccs; 
			image<rgb> *seg = segment_image(input, sigma, k, min_size, &num_ccs);
			image<uchar> *gr = imageRGBtoGRAY(seg);
			int num = imRef(seg,0,0).r; 

			std::ofstream myfile;
			myfile.open("segmented.txt");
			for(int R = 0; R < seg->height(); R++)
			{
				for(int C = 0; C < seg->width(); C++)
				{
					int numR, numG, numB, numGr;
					numR = imRef(seg,C,R).r;
					numG = imRef(seg,C,R).g;
					numB = imRef(seg,C,R).b;
					numGr = imRef(gr,C,R);
					myfile << numR << " " << numG << " " << numB << " " << numGr << std::endl;
				}
			}
			std::string output_location("../images/img.ppm");
			savePPM(seg, output_location.c_str());
			myfile.close();

			int H = 0;
			int W = 0;
			num = im.rows*im.cols;
			std::vector<std::vector<int> > mask;
			mask.resize(im.rows, std::vector<int>(im.cols, 0));
			int maskValue = 0;
			int v = 0;

			std::ifstream infile("segmented.txt");
			int R,G,B,GR;
	
			std::vector<std::vector<int> > mem;
			mem.resize(num, std::vector<int>(3, 0));
		 	std::vector<int> val;
			val.resize(num, 0);
			int insertion_mem_index = 1;
			std::vector<int> query = {R, G, B};

			for(int i = 0; i < num; i++)
			{
				infile >> R >> G >> B >> GR;
				if(i==0)
				{
					mem[i][0] = R;
					mem[i][1] = G;
					mem[i][2] = B;
					val[i] = maskValue;
					mask[H][W] = val[0];
				}
				else
				{
					auto pos = find(mem.begin(), mem.end(), query);
					if(pos != mem.end())
					{
						mask[H][W] = val[pos-mem.begin()];
					}
					else
					{
						mem[insertion_mem_index][0] = R;
						mem[insertion_mem_index][1] = G;
						mem[insertion_mem_index][2] = B;
						maskValue++;
						insertion_mem_index++;
						val[insertion_mem_index] = maskValue;
						mask[H][W] = maskValue;				
					}
				}
				W = W + 1;
				if(W == im.cols)
				{
					H = H + 1;
					W = 0;
				}
			}

			totalMasks = maskValue;
			return mask;
		}


		void SelectiveSearchBase::init()
		{
		}

		shared_ptr<Detections2D> SelectiveSearchBase::detectOmni(shared_ptr<SceneImage> scene)
		{
			return make_shared<Detections2D>();	
		}

		shared_ptr<Detections> SelectiveSearchBase::detect(shared_ptr<SceneImage> scene)
		{
			return make_shared<Detections>();		
		}

	}
}
		
