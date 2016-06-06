#include "od/detectors/local2D/simple_ransac_detection/CsvWriter.h"

namespace od {
  
  namespace l2d {

    CsvWriter::CsvWriter(const std::string & path, const std::string & separator){
      _file_.open(path.c_str(), std::ofstream::out);
      _is_first_term_ = true;
      _separator_ = separator;
    }

    CsvWriter::~CsvWriter() {
      _file_.flush();
      _file_.close();
    }

    void CsvWriter::writeXYZ(const std::vector<cv::Point3f> & list_points3d)
    {
      std::string x, y, z;
      for(size_t i = 0; i < list_points3d.size(); ++i)
      {
        x = std::to_string(list_points3d[i].x);
        y = std::to_string(list_points3d[i].y);
        z = std::to_string(list_points3d[i].z);

        _file_ << x << _separator_ << y << _separator_ << z << std::endl;
      }

    }

    void CsvWriter::writeUVXYZ(const std::vector<cv::Point3f> & list_points3d, const std::vector<cv::Point2f> & list_points2d, const cv::Mat & descriptors)
    {
      std::string u, v, x, y, z, descriptor_str;
      for(size_t i = 0; i < list_points3d.size(); ++i)
      {
        u = std::to_string(list_points2d[i].x);
        v = std::to_string(list_points2d[i].y);
        x = std::to_string(list_points3d[i].x);
        y = std::to_string(list_points3d[i].y);
        z = std::to_string(list_points3d[i].z);

        _file_ << u << _separator_ << v << _separator_ << x << _separator_ << y << _separator_ << z;

        for(size_t j = 0; j < 32; ++j)
        {
          descriptor_str = std::to_string(descriptors.at<float>(i,j));
          _file_ << _separator_ << descriptor_str;
        }
        _file_ << std::endl;
      }
    }

  }
  
}