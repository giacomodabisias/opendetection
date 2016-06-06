#pragma once
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <string>
#include <stdlib.h>
#include "od/detectors/local2D/simple_ransac_detection/Utils.h"


namespace od {
  
  namespace l2d {

    class CsvReader {
    public:
      /**
      * The default constructor of the CSV reader Class.
      * The default separator is ' ' (empty space)
      *
      * @param path - The path of the file to read
      * @param separator - The separator character between words per line
      * @return
      */
      CsvReader(const std::string & path, const char & separator = ' ');

      /**
      * Read a plane text file with .ply format
      *
      * @param list_vertex - The container of the vertices list of the mesh
      * @param list_triangles - The container of the triangles list of the mesh
      * @return
      */
      void readPLY(std::vector<cv::Point3f> & list_vertex, std::vector<std::vector<int> > & list_triangles);

    private:
      /** The current stream file for the reader */
      std::ifstream _file_;
      /** The separator character between words for each line */
      char _separator_;
    };

  }
  
}
