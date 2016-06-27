#include "od/detectors/misc/detection/ODDetectorMultiAlgo.hpp"


namespace od
{

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZ>::detectOmni(shared_ptr<ODScene> scene);

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZ>::detect(shared_ptr<ODScene> scene);

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZ>::detect(shared_ptr<ODSceneImage> scene);

  template
  shared_ptr<ODDetections2D> ODDetectorMultiAlgo2D<pcl::PointXYZ>::detectOmni(shared_ptr<ODSceneImage > scene);

  template
  void ODDetectorMultiAlgo2D<pcl::PointXYZ>::init();



  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGB>::detectOmni(shared_ptr<ODScene> scene);

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGB>::detect(shared_ptr<ODScene> scene);

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGB>::detect(shared_ptr<ODSceneImage> scene);

  template
  shared_ptr<ODDetections2D> ODDetectorMultiAlgo2D<pcl::PointXYZRGB>::detectOmni(shared_ptr<ODSceneImage > scene);

  template
  void ODDetectorMultiAlgo2D<pcl::PointXYZRGB>::init();



  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ODScene> scene);

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGBA>::detect(shared_ptr<ODScene> scene);

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo2D<pcl::PointXYZRGBA>::detect(shared_ptr<ODSceneImage> scene);

  template
  shared_ptr<ODDetections2D> ODDetectorMultiAlgo2D<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ODSceneImage > scene);

  template
  void ODDetectorMultiAlgo2D<pcl::PointXYZRGBA>::init();



  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZ>::detectOmni(shared_ptr<ODScene> scene);

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZ>::detect(shared_ptr<ODScene> scene);

  template
  void ODDetectorMultiAlgo<pcl::PointXYZ>::init();

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZ>::detect(shared_ptr<ODScenePointCloud<pcl::PointXYZ> > scene);

  template
  shared_ptr<ODDetections3D> ODDetectorMultiAlgo<pcl::PointXYZ>::detectOmni(shared_ptr<ODScenePointCloud<pcl::PointXYZ> > scene);



  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGB>::detectOmni(shared_ptr<ODScene> scene);

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGB>::detect(shared_ptr<ODScene> scene);

  template
  void ODDetectorMultiAlgo<pcl::PointXYZRGB>::init();

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGB>::detect(shared_ptr<ODScenePointCloud<pcl::PointXYZRGB> > scene);

  template
  shared_ptr<ODDetections3D> ODDetectorMultiAlgo<pcl::PointXYZRGB>::detectOmni(shared_ptr<ODScenePointCloud<pcl::PointXYZRGB> > scene);



  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ODScene> scene);

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGBA>::detect(shared_ptr<ODScene> scene);

  template
  void ODDetectorMultiAlgo<pcl::PointXYZRGBA>::init();

  template
  shared_ptr<ODDetections> ODDetectorMultiAlgo<pcl::PointXYZRGBA>::detect(shared_ptr<ODScenePointCloud<pcl::PointXYZRGBA> > scene);

  template
  shared_ptr<ODDetections3D> ODDetectorMultiAlgo<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ODScenePointCloud<pcl::PointXYZRGBA> > scene);

}