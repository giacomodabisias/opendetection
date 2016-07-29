/*
 * Mesh.cpp
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#include "od/detectors/local2D/simple_ransac_detection/Mesh.h"

namespace od {
  
  namespace l2d {

    // --------------------------------------------------- //
    //                   TRIANGLE CLASS                    //
    // --------------------------------------------------- //

    /**  The custom constructor of the Triangle Class */
    Triangle::Triangle(int id, const cv::Point3f & V0, const cv::Point3f & V1, const cv::Point3f & V2)
    {
      id_ = id; v0_ = V0; v1_ = V1; v2_ = V2;
    }

    cv::Point3f Triangle::getV0() const {
     return v0_; 
    }
    cv::Point3f Triangle::getV1() const { 
     return v1_; 
    }
    cv::Point3f Triangle::getV2() const {
     return v2_; 
    }



    // --------------------------------------------------- //
    //                     RAY CLASS                       //
    // --------------------------------------------------- //

    /**  The custom constructor of the Ray Class */
    Ray::Ray(const cv::Point3f & P0, const cv::Point3f & P1) {
      p0_ = P0; p1_ = P1;
    }

    cv::Point3f Ray::getP0()
    { 
      return p0_; 
    }
    cv::Point3f Ray::getP1()
    {
     return p1_; 
    }



    // --------------------------------------------------- //
    //                 OBJECT MESH CLASS                   //
    // --------------------------------------------------- //

    /** The default constructor of the ObjectMesh Class */
    Mesh::Mesh() : list_vertex_(0) , list_triangles_(0)
    {
      id_ = 0;
      num_vertexs_ = 0;
      num_triangles_ = 0;
    }



    std::vector<std::vector<int> > Mesh::getTrianglesList() const 
    { 
      return list_triangles_; 
    }

    std::vector<cv::Point3f> Mesh::getVertices()  
    { 
      return list_vertex_; 
    }

    cv::Point3f Mesh::getVertex(int pos) const 
    { 
      return list_vertex_[pos]; 
    }

    int Mesh::getNumVertices() const 
    {
     return num_vertexs_; 
    }

    /** Load a CSV with *.ply format **/
    void Mesh::load(const std::string & path)
    {

      // Create the reader
      CsvReader csvReader(path);

      // Clear previous data
      list_vertex_.clear();
      list_triangles_.clear();

      // Read from .ply file
      csvReader.readPLY(list_vertex_, list_triangles_);

      // Update mesh attributes
      num_vertexs_ = (int)list_vertex_.size();
      num_triangles_ = (int)list_triangles_.size();

    }
    
  }

}