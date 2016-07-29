/*
 * Mesh.h
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include "od/detectors/local2D/simple_ransac_detection/CsvReader.h"

namespace od {
  
  namespace l2d {
    // --------------------------------------------------- //
    //                 TRIANGLE CLASS                      //
    // --------------------------------------------------- //

    class Triangle {
    public:

      explicit Triangle(int id, const cv::Point3f & V0, const cv::Point3f & V1, const cv::Point3f & V2);

      cv::Point3f getV0() const;
      cv::Point3f getV1() const;
      cv::Point3f getV2() const;

    private:
      /** The identifier number of the triangle */
      int id_;
      /** The three vertices that defines the triangle */
      cv::Point3f v0_, v1_, v2_;
    };


    // --------------------------------------------------- //
    //                     RAY CLASS                       //
    // --------------------------------------------------- //

    class Ray {
    public:

      explicit Ray(const cv::Point3f & P0, const cv::Point3f & P1);

      cv::Point3f getP0();
      cv::Point3f getP1();

    private:
      /** The two points that defines the ray */
      cv::Point3f p0_, p1_;
    };


    // --------------------------------------------------- //
    //                OBJECT MESH CLASS                    //
    // --------------------------------------------------- //

    class Mesh
    {
    public:

      Mesh();

      std::vector<std::vector<int> > getTrianglesList() const;
      std::vector<cv::Point3f> getVertices();
      cv::Point3f getVertex(int pos) const;
      int getNumVertices() const;

      void load(const std::string & path_file);

    private:
      /** The identification number of the mesh */
      int id_;
      /** The current number of vertices in the mesh */
      int num_vertexs_;
      /** The current number of triangles in the mesh */
      int num_triangles_;
      /* The list of triangles of the mesh */
      std::vector<cv::Point3f> list_vertex_;
      /* The list of triangles of the mesh */
      std::vector<std::vector<int> > list_triangles_;
    };

  }

}
