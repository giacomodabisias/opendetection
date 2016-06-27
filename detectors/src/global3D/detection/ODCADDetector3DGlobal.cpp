#include "od/detectors/global3D/detection/ODCADDetector3DGlobal.hpp"
#include "od/common/pipeline/ODDetection.h"

namespace od {

	  namespace g3d { 

		template
	    shared_ptr<ODDetections> ODCADDetector3DGlobal<pcl::PointXYZ>::detect(shared_ptr<ODScene> scene);
	
	    template
	    shared_ptr<ODDetections> ODCADDetector3DGlobal<pcl::PointXYZ>::detectOmni(shared_ptr<ODScene> scene);

	    template
	    void ODCADDetector3DGlobal<pcl::PointXYZ>::init();

	    template
	    shared_ptr<ODDetections3D> ODCADDetector3DGlobal<pcl::PointXYZ>::detectOmni(shared_ptr<ODScenePointCloud<pcl::PointXYZ> > scene);

	    template
	    shared_ptr<ODDetections> ODCADDetector3DGlobal<pcl::PointXYZ>::detect(shared_ptr<ODScenePointCloud<pcl::PointXYZ> > scene);



		template
	    shared_ptr<ODDetections> ODCADDetector3DGlobal<pcl::PointXYZRGB>::detect(shared_ptr<ODScene> scene);

	    template
	    shared_ptr<ODDetections> ODCADDetector3DGlobal<pcl::PointXYZRGB>::detectOmni(shared_ptr<ODScene> scene);

	    template
	    void ODCADDetector3DGlobal<pcl::PointXYZRGB>::init();

	    template
	    shared_ptr<ODDetections3D> ODCADDetector3DGlobal<pcl::PointXYZRGB>::detectOmni(shared_ptr<ODScenePointCloud<pcl::PointXYZRGB> > scene);

	    template
	    shared_ptr<ODDetections> ODCADDetector3DGlobal<pcl::PointXYZRGB>::detect(shared_ptr<ODScenePointCloud<pcl::PointXYZRGB> > scene);



		template
	    shared_ptr<ODDetections> ODCADDetector3DGlobal<pcl::PointXYZRGBA>::detect(shared_ptr<ODScene> scene);

	    template
	    shared_ptr<ODDetections> ODCADDetector3DGlobal<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ODScene> scene);

	    template
	    void ODCADDetector3DGlobal<pcl::PointXYZRGBA>::init();

	    template
	    shared_ptr<ODDetections3D> ODCADDetector3DGlobal<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ODScenePointCloud<pcl::PointXYZRGBA> > scene);

	    template
	    shared_ptr<ODDetections> ODCADDetector3DGlobal<pcl::PointXYZRGBA>::detect(shared_ptr<ODScenePointCloud<pcl::PointXYZRGBA> > scene);

	}

}