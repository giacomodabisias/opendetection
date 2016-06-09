#include "od/detectors/local2D/simple_ransac_detection/CsvReader.h"

namespace od {
  
  namespace l2d {

        /** The default constructor of the CSV reader Class */
        CsvReader::CsvReader(const std::string & path, const std::string & separator){
            file_.open(path.c_str(), std::ifstream::in);
            separator_ = separator;
        }

        /* Read a plane text file with .ply format */
        void CsvReader::readPLY(std::vector<cv::Point3f> & list_vertex, std::vector<std::vector<int> > & list_triangles)
        {
            std::string line, tmp_str, n;
            int num_vertex = 0, num_triangles = 0;
            int count = 0;
            bool end_header = false;
            bool end_vertex = false;

            // Read the whole *.ply file
            while(getline(file_, line)) {
                std::stringstream liness(line);

            // read header
            if(!end_header)
            {
                getline(liness, tmp_str, separator_.c_str());
                if(tmp_str == "element")
                {
                    getline(liness, tmp_str, separator_.c_str());
                    getline(liness, n);
                    if(tmp_str == "vertex") 
                        num_vertex = std::stoi(n);
                    if(tmp_str == "face") 
                        num_triangles = std::stoi(n);
                }
                if(tmp_str == "end_header") 
                    end_header = true;
            }

            // read file content
            else if(end_header)
            {
                 // read vertex and add into 'list_vertex'
                 if(!end_vertex && count < num_vertex)
                 {
                     std::string x, y, z;
                     getline(liness, x, separator_.c_str());
                     getline(liness, y, _separator_);
                     getline(liness, z);

                     cv::Point3f tmp_p;
                     tmp_p.x = atof(x.c_str());
                     tmp_p.y = atof(y.c_str());
                     tmp_p.z = atof(z.c_str());
                     list_vertex.push_back(tmp_p);

                     count++;
                     if(count == num_vertex)
                     {
                         count = 0;
                         end_vertex = !end_vertex;
                     }
                 }
                 // read faces and add into 'list_triangles'
                 else if(end_vertex && count < num_triangles)
                 {
                     std::string num_pts_per_face, id0, id1, id2;
                     getline(liness, num_pts_per_face, _separator_);
                     getline(liness, id0, _separator_);
                     getline(liness, id1, _separator_);
                     getline(liness, id2);

                     std::vector<int> tmp_triangle(3);
                     tmp_triangle[0] = std::stoi(id0);
                     tmp_triangle[1] = std::stoi(id1);
                     tmp_triangle[2] = std::stoi(id2);
                     list_triangles.push_back(tmp_triangle);

                     count++;
              }
            }
          }
        }
    }
}