//
// Created by sarkar on 10.08.15.
//
#pragma once

#include "od/common/pipeline/Detector.h"
#include <pcl/apps/3d_rec_framework/pipeline/global_nn_classifier.h>
#include <pcl/apps/3d_rec_framework/pc_source/mesh_source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/vfh_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/esf_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/cvfh_estimator.h>
#include <pcl/apps/3d_rec_framework/utils/metrics.h>
#include <pcl/apps/dominant_plane_segmentation.h>

namespace od
{
  namespace g3d
  {

    template<typename PointT>
    class CADDetector3DGlobal : public Detector3D<PointT>
    {

    public:
      CADDetector3DGlobal(const std::string & training_data_location = std::string(""), 
                            const std::string & training_input_location = std::string("")) : 
                            Detector3D<PointT>(training_data_location), NN_(2), desc_name_("esf")
      {
        this->trained_location_identifier_ = std::string("GLOBAL3DVFH");
        this->training_input_location_ = training_input_location;
      }

      void init();

      int getNN() const
      {
        return NN_;
      }

      void setNN(int NN)
      {
        NN_ = NN;
      }

      const std::string & getDescName() const
      {
        return desc_name_;
      }

      void setDescName(const std::string & desc_name)
      {
        desc_name_ = desc_name;
      }

      shared_ptr<Detections> detect(shared_ptr<Scene> scene);
      shared_ptr<Detections> detectOmni(shared_ptr<Scene> scene);

      shared_ptr<Detections> detect(shared_ptr<ScenePointCloud<PointT> > scene);
      shared_ptr<Detections3D> detectOmni(shared_ptr<ScenePointCloud<PointT> > scene);

    protected:

      int NN_;
      std::string desc_name_;
      shared_ptr<pcl::rec_3d_framework::GlobalClassifier<pcl::PointXYZ> > global_;

    };

    template<typename PointT>
    shared_ptr<Detections> CADDetector3DGlobal<PointT>::detect(shared_ptr<Scene> scene) 
    {
      std::cout << "not implemented, use with shared_ptr<ScenePointCloud<PointT>>" << std::endl; 
      return nullptr;
    };

    template<typename PointT>
    shared_ptr<Detections> CADDetector3DGlobal<PointT>::detectOmni(shared_ptr<Scene> scene)
    {
      std::cout << "not implemented, use with shared_ptr<ScenePointCloud<PointT>>" << std::endl; 
      return nullptr;
    };

    template<typename PointT>
    void CADDetector3DGlobal<PointT>::init()
    {

      shared_ptr<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > mesh_source(new pcl::rec_3d_framework::MeshSource<pcl::PointXYZ>);
      mesh_source->setPath(this->training_input_location_);
      std::string training_dir_specific = this->getSpecificTrainingDataLocation();
      mesh_source->setModelScale(1.f);
      mesh_source->generate(training_dir_specific);

      shared_ptr<pcl::rec_3d_framework::Source<pcl::PointXYZ> > cast_source;
      cast_source = dynamic_pointer_cast<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> >(mesh_source);

      shared_ptr<pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal> > normal_estimator;
      normal_estimator.reset(new pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal>);
      normal_estimator->setCMR(true);
      normal_estimator->setDoVoxelGrid(true);
      normal_estimator->setRemoveOutliers(true);
      normal_estimator->setFactorsForCMR(3, 7);

      if(desc_name_.compare("vfh") == 0)
      {
        shared_ptr<pcl::rec_3d_framework::VFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> > vfh_estimator;
        vfh_estimator.reset(new pcl::rec_3d_framework::VFHEstimation<pcl::PointXYZ, pcl::VFHSignature308>);
        vfh_estimator->setNormalEstimator(normal_estimator);

        shared_ptr<pcl::rec_3d_framework::GlobalEstimator<pcl::PointXYZ, pcl::VFHSignature308> > cast_estimator;
        cast_estimator = dynamic_pointer_cast<pcl::rec_3d_framework::VFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> >(
            vfh_estimator);

        shared_ptr<pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::VFHSignature308> > global(
            new pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::VFHSignature308>());
        global->setDataSource(cast_source);
        global->setTrainingDir(training_dir_specific);
        global->setDescriptorName(desc_name_);
        global->setNN(NN_);
        global->setFeatureEstimator(cast_estimator);
        global->initialize(false);
        global_ = global;

      } else if(desc_name_.compare("cvfh") == 0)
      {
        shared_ptr<pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> > vfh_estimator;
        vfh_estimator.reset(new pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308>);
        vfh_estimator->setNormalEstimator(normal_estimator);

        shared_ptr<pcl::rec_3d_framework::GlobalEstimator<pcl::PointXYZ, pcl::VFHSignature308> > cast_estimator;
        cast_estimator = dynamic_pointer_cast<pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> >(
            vfh_estimator);

        shared_ptr<pcl::rec_3d_framework::GlobalNNPipeline<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::VFHSignature308> > global(
          new pcl::rec_3d_framework::GlobalNNPipeline<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::VFHSignature308>());
        global->setDataSource(cast_source);
        global->setTrainingDir(training_dir_specific);
        global->setDescriptorName(desc_name_);
        global->setFeatureEstimator(cast_estimator);
        global->setNN(NN_);
        global->initialize(false);
        global_ = global;
      } else if(desc_name_.compare("esf") == 0)
      {
        shared_ptr<pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> > estimator;
        estimator = make_shared<pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> >();

        shared_ptr<pcl::rec_3d_framework::GlobalEstimator<pcl::PointXYZ, pcl::ESFSignature640> > cast_estimator;
        cast_estimator = dynamic_pointer_cast<pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> >(
            estimator);

        shared_ptr<pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::ESFSignature640> > global(
            new pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::ESFSignature640>());
        global->setDataSource(cast_source);
        global->setTrainingDir(training_dir_specific);
        global->setDescriptorName(desc_name_);
        global->setFeatureEstimator(cast_estimator);
        global->setNN(NN_);
        global->initialize(false);
        global_ = global;
      } else
      {
        std::cout << "FATAL: descriptor type not available." << std::endl;
      }

    }

    template<typename PointT>
    shared_ptr<Detections3D> CADDetector3DGlobal<PointT>::detectOmni(shared_ptr<ScenePointCloud<PointT> > scene)
    {
      shared_ptr<Detections3D> detections = make_shared<Detections3D>();

      typename pcl::PointCloud<PointT>::Ptr frame = scene->getPointCloud();
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*frame, *xyz_points);

      //Step 1 -> Segment
      pcl::apps::DominantPlaneSegmentation<pcl::PointXYZ> dps;
      dps.setInputCloud(xyz_points);
      dps.setMaxZBounds(1.25f);
      dps.setObjectMinHeight(0.005);
      dps.setMinClusterSize(1000);
      dps.setWSize(9);
      dps.setDistanceBetweenClusters(0.1f);

      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
      std::vector<pcl::PointIndices> indices;
      dps.setDownsamplingSize(0.02f);
      dps.compute_fast(clusters);
      dps.getIndicesClusters(indices);
      Eigen::Vector4f table_plane_;
      dps.getTableCoefficients(table_plane_);

      for(size_t i = 0; i < clusters.size(); ++i)
      {

        global_->setInputCloud(xyz_points);
        global_->setIndices(indices[i].indices);
        global_->classify();

        std::vector<std::string> categories;
        std::vector<float> conf;
        global_->getCategory(categories);
        global_->getConfidence(conf);
        //detection done!

        std::string category = categories[0];
        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(*xyz_points, indices[i].indices, centroid);
        //position at 3D identified!

        //now fill up the detection:
        shared_ptr<Detection3D> detection = make_shared<Detection3D>();
        detection->setType(Detection::CLASSIFICATION);
        detection->setId(categories[0]);
        detection->setLocation(centroid);
        detection->setMetainfoCluster(clusters[i]);
        detections->push_back(detection);
      }

      return detections;
    }

    template<typename PointT>
    shared_ptr<Detections> CADDetector3DGlobal<PointT>::detect(shared_ptr<ScenePointCloud<PointT> > scene)
    {
      shared_ptr<Detections> detections = make_shared<Detections>();

      typename pcl::PointCloud<PointT>::Ptr frame = scene->getPointCloud();
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*frame, *xyz_points);

      global_->setInputCloud(xyz_points);
      //no indices set, so it would try to classify the entire PC
      global_->classify();

      std::vector<std::string> categories;
      std::vector<float> conf;
      global_->getCategory(categories);
      global_->getConfidence(conf);
      //detection done!

      //now fill up the detection:
      shared_ptr<Detection> detection(new Detection(Detection::CLASSIFICATION, categories[0], conf[0]));

      detections->push_back(detection);

      return detections;
    }

#ifndef DOXYGEN_SHOULD_SKIP_THIS
    
    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZ>::detect(shared_ptr<Scene> scene);

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZ>::detectOmni(shared_ptr<Scene> scene);

    extern template
    void CADDetector3DGlobal<pcl::PointXYZ>::init();

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZ>::detect(shared_ptr<Scene> scene);

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZ>::detectOmni(shared_ptr<Scene> scene);

    extern template
    shared_ptr<Detections3D> CADDetector3DGlobal<pcl::PointXYZ>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZ> > scene);

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZ>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZ> > scene);



    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGB>::detect(shared_ptr<Scene> scene);

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGB>::detectOmni(shared_ptr<Scene> scene);

    extern template
    void CADDetector3DGlobal<pcl::PointXYZRGB>::init();

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGB>::detect(shared_ptr<Scene> scene);

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGB>::detectOmni(shared_ptr<Scene> scene);

    extern template
    shared_ptr<Detections3D> CADDetector3DGlobal<pcl::PointXYZRGB>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZRGB> > scene);

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGB>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZRGB> > scene);



    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGBA>::detect(shared_ptr<Scene> scene);

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGBA>::detectOmni(shared_ptr<Scene> scene);

    extern template
    void CADDetector3DGlobal<pcl::PointXYZRGBA>::init();

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGBA>::detect(shared_ptr<Scene> scene);

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGBA>::detectOmni(shared_ptr<Scene> scene);

    extern template
    shared_ptr<Detections3D> CADDetector3DGlobal<pcl::PointXYZRGBA>::detectOmni(shared_ptr<ScenePointCloud<pcl::PointXYZRGBA> > scene);

    extern template
    shared_ptr<Detections> CADDetector3DGlobal<pcl::PointXYZRGBA>::detect(shared_ptr<ScenePointCloud<pcl::PointXYZRGBA> > scene);

#endif
  }
}
