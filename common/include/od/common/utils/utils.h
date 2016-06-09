//
// Created by sarkar on 09.06.15.
//

#pragma once
#include <boost/preprocessor.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <opencv2/core/core.hpp>

#include <fstream>
#include <iostream>
#include <glob.h>

#define X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE(r, data, elem)    \
    case elem : return BOOST_PP_STRINGIZE(elem);

#define OD_DEFINE_ENUM_WITH_STRING_CONVERSIONS(name, enumerators)                \
    enum name {                                                               \
        BOOST_PP_SEQ_ENUM(enumerators)                                        \
    };                                                                        \
                                                                              \
    inline const char * enumToString(name v)                                       \
    {                                                                         \
        switch (v)                                                            \
        {                                                                     \
            BOOST_PP_SEQ_FOR_EACH(                                            \
                X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE,          \
                name,                                                         \
                enumerators                                                   \
            )                                                                 \
            default: return "[Unknown " BOOST_PP_STRINGIZE(name) "]";         \
        }                                                                     \
    }


namespace od
{

  //TODO REMOVE AND SUBSTITUTE USING BOOST FILESYSTEM
  std::vector<std::string> myglob(const std::string & pat);

  void normL2(cv::Mat &descriptors);

  /**
    * @brief Makes composite image from the given images. Equal number of rows and columns are preferred. Therefore, number of rows = sqrt(number of input images)
    *
    * @param imgs Vector of Images.
    * @param cellSize Size of individual images to be placed inside the composite images. images from `imgs` will be resized to this size before appending.
    * @param messages Messages to be put on the top left of each image in `imgs`. Note `message.size()` should be equal to `imgs.size()`.
    * @return new composite image.
    */
  cv::Mat makeCanvasMultiImages(const std::vector<cv::Mat> & imgs, const cv::Size & cellSize, const std::vector<std::string> & messages);

  cv::Scalar getHashedColor(const std::string & name, int offset = 100);

  std::string getTexfileinObj(const std::string & objfilename);

  namespace fileutils {

      std::string getFirstFile(const std::string & base_path, const std::string & extension);

      void getFilesInDirectoryRec(const std::string & base_path, const std::string & extension, std::vector<std::string> & files);
      void getFilesInDirectoryRec(const std::string & base_path, const std::vector<std::string> & extensions, std::vector<std::string> & files);

      void getFilesInDirectoryInternal(const boost::filesystem::path & dir, const std::string & rel_path_so_far, std::vector<std::string> & relative_paths, const std::vector<std::string> & exts);
      void getFilesInDirectoryInternal(const boost::filesystem::path & dir, const std::string & rel_path_so_far, std::vector<std::string> & relative_paths, const std::string & ext);
      
      void getFilesInDirectory(const boost::filesystem::path & dir, const std::string & rel_path_so_far, std::vector<std::string> & relative_paths, const std::string & ext);
      void createTrainingDir(const std::string & training_dir);

      //TODO REMOVE
      void getArgvArgc(std::string const & commandline, char ***argv, int & argc);
  }
}
