#include "od/detectors/global3D/detection/CADDetector3DGlobal.hpp"
#include "od/common/pipeline/Detection.h"

namespace od {

	  namespace g3d { 

		template
	    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZ>::detect(shared_ptr<Scene> scene);
	
	    template
	    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZ>::detectOmni(shared_ptr<Scene> scene);

	    template
	    void CADDetector3DGlobal<pcl::PointXYZ>::init();

	    template
	    shared_ptr<Detections3D> CADDetector3DGlobal<pcl::PointXYZ>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZ> > scene);

	    template
	    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZ>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZ> > scene);



		template
	    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGB>::detect(shared_ptr<Scene> scene);

	    template
	    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGB>::detectOmni(shared_ptr<Scene> scene);

	    template
	    void CADDetector3DGlobal<pcl::PointXYZRGB>::init();

	    template
	    shared_ptr<Detections3D> CADDetector3DGlobal<pcl::PointXYZRGB>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZRGB> > scene);

	    template
	    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGB>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZRGB> > scene);



		template
	    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGBA>::detect(shared_ptr<Scene> scene);

	    template
	    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGBA>::detectOmni(shared_ptr<Scene> scene);

	    template
	    void CADDetector3DGlobal<pcl::PointXYZRGBA>::init();

	    template
	    shared_ptr<Detections3D> CADDetector3DGlobal<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZRGBA> > scene);

	    template
	    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGBA>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZRGBA> > scene);

	}

}