#include "od/common/utils/ODViewer.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char * argv[]){
	
	if(argc < 3){
		std::cout << "Use pointcloud and image as input" << std::endl;
		return -1;
	}

	shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) == -1){
    	std::cout << "Could not load " << argv[1] << std::endl;
    	return -1;
    }

    od::ODViewer viewer;
    viewer.render(cloud, std::string("test_cloud"));

    while(!viewer.toStop()){
    	viewer.spin();
    }

    cv::Mat image = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);   
	viewer.render(image, std::string("test_image"));

    while(!viewer.toStop()){
    	viewer.spin();
    }

    viewer.render(cloud, std::string("test_cloud2"));

    while(!viewer.toStop()){
    	viewer.spin();
    }

    //This will fail since test_cloud has been overwritten by test_cloud2
    viewer.update(cloud, std::string("test_cloud"));

    while(!viewer.toStop()){
    	viewer.spin();
    }

	return 0;
}