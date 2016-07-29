#include "od/detectors/misc/detection/DetectorMultiAlgo.hpp"


namespace od
{

  template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZ>::detectOmni(shared_ptr<Scene> scene);

  template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZ>::detect(shared_ptr<Scene> scene);

  template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZ>::detect(shared_ptr<SceneImage> scene);

  template
  shared_ptr<Detections2D> DetectorMultiAlgo2D<pcl::PointXYZ>::detectOmni(shared_ptr<SceneImage > scene);

  template
  void DetectorMultiAlgo2D<pcl::PointXYZ>::init();



  template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGB>::detectOmni(shared_ptr<Scene> scene);

  template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGB>::detect(shared_ptr<Scene> scene);

  template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGB>::detect(shared_ptr<SceneImage> scene);

  template
  shared_ptr<Detections2D> DetectorMultiAlgo2D<pcl::PointXYZRGB>::detectOmni(shared_ptr<SceneImage > scene);

  template
  void DetectorMultiAlgo2D<pcl::PointXYZRGB>::init();



  template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGBA>::detectOmni(shared_ptr<Scene> scene);

  template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGBA>::detect(shared_ptr<Scene> scene);

  template
  shared_ptr<Detections> DetectorMultiAlgo2D<pcl::PointXYZRGBA>::detect(shared_ptr<SceneImage> scene);

  template
  shared_ptr<Detections2D> DetectorMultiAlgo2D<pcl::PointXYZRGBA>::detectOmni(shared_ptr<SceneImage > scene);

  template
  void DetectorMultiAlgo2D<pcl::PointXYZRGBA>::init();



  template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZ>::detectOmni(shared_ptr<Scene> scene);

  template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZ>::detect(shared_ptr<Scene> scene);

  template
  void DetectorMultiAlgo<pcl::PointXYZ>::init();

  template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZ>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZ> > scene);

  template
  shared_ptr<Detections3D> DetectorMultiAlgo<pcl::PointXYZ>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZ> > scene);



  template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGB>::detectOmni(shared_ptr<Scene> scene);

  template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGB>::detect(shared_ptr<Scene> scene);

  template
  void DetectorMultiAlgo<pcl::PointXYZRGB>::init();

  template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGB>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZRGB> > scene);

  template
  shared_ptr<Detections3D> DetectorMultiAlgo<pcl::PointXYZRGB>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZRGB> > scene);



  template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGBA>::detectOmni(shared_ptr<Scene> scene);

  template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGBA>::detect(shared_ptr<Scene> scene);

  template
  void DetectorMultiAlgo<pcl::PointXYZRGBA>::init();

  template
  shared_ptr<Detections> DetectorMultiAlgo<pcl::PointXYZRGBA>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZRGBA> > scene);

  template
  shared_ptr<Detections3D> DetectorMultiAlgo<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZRGBA> > scene);

}