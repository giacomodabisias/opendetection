//
// Created by sarkar on 19.06.15.
//
#include "od/common/utils/ODUtils.h"


namespace od
{

  std::vector<std::string> myglob(const std::string & pat)
  {
    glob_t glob_result;
    glob(pat.c_str(), GLOB_TILDE, nullptr, &glob_result);
    std::vector<std::string> ret;
    for(size_t i = 0; i < glob_result.gl_pathc; ++i) {
      ret.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return ret;
  }

  void normL2(cv::Mat & descriptors)
  {
    for (int r = 0; r < descriptors.rows; r++)
    {
      float norm = 0;
      for (size_t c = 0; c < descriptors.cols; c++) 
        norm += (descriptors.at<float>(r, c)*descriptors.at<float>(r, c));

      norm = 1.0/sqrt(norm);

      for (size_t c = 0; c < descriptors.cols; c++) 
        descriptors.at<float>(r, c) *= norm;
    }
  }

  void drawTextTopLeft(cv::Mat & image, const std::string & text, const cv::Scalar & color)
  {
    cv::putText(image, text, cv::Point(25,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, color, 2, 16);
  }

  cv::Mat makeCanvasMultiImages(const std::vector<cv::Mat> & imgs, const cv::Size & cell_size, const std::vector<std::string> & messages)
  {

    bool printmssg = false;
    if(messages.size() == imgs.size()) 
      printmssg = true;

    const float n_imgs = imgs.size();
    const int imgs_in_row = ceil(sqrt(n_imgs));     
    const int imgs_in_col = ceil(n_imgs/imgs_in_row); 

    const unsigned int cell_width = cell_size.width;
    const unsigned int cell_height = cell_size.height;

    const int resultImgW = cell_width * imgs_in_row;
    const int resultImgH = cell_height * imgs_in_col;

    cv::Mat result_img = cv::Mat::zeros(resultImgH, resultImgW, CV_8UC3);
    unsigned int ind = 0;
    cv::Mat tmp;
    for(size_t i = 0; i < imgs_in_col; i++)
    {
      for(size_t j = 0; j < imgs_in_row; j++)
      {
        if(ind < imgs.size())
        {
          const int cell_row = i*cell_height;
          const int cell_col = j*cell_width;

          resize(imgs[ind], tmp, cell_size);

          if(printmssg) 
            drawTextTopLeft(tmp, messages[ind], cv::Scalar(0, 255, 255));

          tmp.copyTo(result_img(cv::Range(cell_row, cell_row+cell_height), cv::Range(cell_col, cell_col+cell_width)));
        }
        ind++;
      }
    }

    return result_img;
  }

  std::string getTexfileinObj(const std::string & objfilename)
  {

    std::string input_dir(boost::filesystem::path(objfilename).parent_path().string());

    std::ifstream input(objfilename);
    std::string line;
    while(getline(input, line))
    {
      std::istringstream iss(line);
      std::string tok1;
      iss >> tok1;
      if(tok1 == "mtllib")
      {
        std::string tok2;

        iss >> tok2;
        std::string linemtl;

        std::ifstream inputmtl((input_dir + "/" + tok2));
        while(getline(inputmtl, linemtl))
        {
          std::istringstream issmtl(linemtl);
          issmtl >> tok1;
          if(tok1 == "map_Kd")
          {
            issmtl >> tok2;
            return input_dir + "/" + tok2;
          }
        }
      }
    }
    return "";
  }

  cv::Scalar getHashedColor(const std::string & name, int offset)
  {
    boost::hash<std::string> string_hash;
    const int hashed = string_hash(name);

    return CV_RGB((hashed + offset) % 255, (hashed + 2*offset) % 255, (hashed + 3*offset) % 255);
  }


  namespace fileutils {

    std::string getFirstFile(const std::string & base_path, const std::string & extension)
    {
      std::vector<std::string> files;
      fileutils::getFilesInDirectoryInternal(base_path, "", files, extension);
      if(files.size() == 0)
      {
        std::cout << "No file with extension " << extension << " present!" << std::endl;
        std::cout << "Returning empty string" << std::endl;
        return std::string("");
      }
      return files[0];
    }

    void getFilesInDirectoryRec(const std::string & base_path, const std::string & extension, std::vector<std::string> & files)
    {
      fileutils::getFilesInDirectoryInternal(base_path, "", files, extension);
    }

    void getFilesInDirectoryRec(const std::string & base_path, const std::vector<std::string> & extensions, std::vector<std::string> & files)
    {
      fileutils::getFilesInDirectoryInternal(base_path, "", files, extensions);
    }

    void getFilesInDirectoryInternal(const boost::filesystem::path & dir, const std::string & rel_path_so_far, std::vector<std::string> & relative_paths, const std::vector<std::string> & exts)
    {
      boost::filesystem::directory_iterator end_itr;
      for(boost::filesystem::directory_iterator itr(dir); itr != end_itr; ++itr) {
        //check if its a directory, then get models in it
        if(boost::filesystem::is_directory(*itr)) {
#if BOOST_FILESYSTEM_VERSION == 3
          std::string so_far = rel_path_so_far + (itr->path().filename()).string() + "/";
#else
          std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

          boost::filesystem::path curr_path = itr->path();
          getFilesInDirectoryInternal(curr_path, so_far, relative_paths, exts);
        } else {
          //check that it is a ply file and then add, otherwise ignore..
          std::vector<std::string> strs;
#if BOOST_FILESYSTEM_VERSION == 3
          std::string file = (itr->path().filename()).string();
#else
          std::string file = (itr->path ()).filename ();
#endif

          boost::split(strs, file, boost::is_any_of("."));
          std::string extension = strs[strs.size() - 1];

          bool flagfound = false;
          for (size_t exti = 0; exti < exts.size(); ++exti)
            if(file.rfind(exts[exti]) != std::string::npos)
            { flagfound = true; 
              break; 
            }

          if(flagfound == true)
          {
#if BOOST_FILESYSTEM_VERSION == 3
            std::string path = rel_path_so_far + (itr->path().filename()).string();
#else
            std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif
            std::string fullpath = rel_path_so_far + itr->path().string();
            relative_paths.push_back(fullpath);
          }
        }
      }
    }

    void getFilesInDirectoryInternal(const boost::filesystem::path & dir, const std::string & rel_path_so_far, std::vector<std::string> & relative_paths, const std::string & ext)
    {
      boost::filesystem::directory_iterator end_itr;
      for(boost::filesystem::directory_iterator itr(dir); itr != end_itr; ++itr) {
        //check if its a directory, then get models in it
        if(boost::filesystem::is_directory(*itr)) {
#if BOOST_FILESYSTEM_VERSION == 3
          std::string so_far = rel_path_so_far + (itr->path().filename()).string() + "/";
#else
          std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

          boost::filesystem::path curr_path = itr->path();
          getFilesInDirectoryInternal(curr_path, so_far, relative_paths, ext);
        } else {
          //check that it is a ply file and then add, otherwise ignore..
          std::vector<std::string> strs;
#if BOOST_FILESYSTEM_VERSION == 3
          std::string file = (itr->path().filename()).string();
#else
            std::string file = (itr->path ()).filename ();
#endif

          boost::split(strs, file, boost::is_any_of("."));
          std::string extension = strs[strs.size() - 1];

          if( file.rfind(ext) != std::string::npos )
          {
#if BOOST_FILESYSTEM_VERSION == 3
            std::string path = rel_path_so_far + (itr->path().filename()).string();
#else
              std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif
            std::string fullpath = rel_path_so_far + itr->path().string();
            relative_paths.push_back(fullpath);
          }
        }
      }
    }

    void getFilesInDirectory(const boost::filesystem::path & dir, const std::string & rel_path_so_far, std::vector<std::string> & relative_paths, const std::string & ext)
    {
      boost::filesystem::directory_iterator end_itr;
      for(boost::filesystem::directory_iterator itr(dir); itr != end_itr; ++itr) {
        //check if its a directory, then get models in it
        if(boost::filesystem::is_directory(*itr)) {
#if BOOST_FILESYSTEM_VERSION == 3
          std::string so_far = rel_path_so_far + (itr->path().filename()).string() + "/";
#else
            std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

          boost::filesystem::path curr_path = itr->path();
          getFilesInDirectoryInternal(curr_path, so_far, relative_paths, ext);
        } else {
          //check that it is a ply file and then add, otherwise ignore..
          std::vector<std::string> strs;
#if BOOST_FILESYSTEM_VERSION == 3
          std::string file = (itr->path().filename()).string();
#else
          std::string file = (itr->path ()).filename ();
#endif

          boost::split(strs, file, boost::is_any_of("."));
          std::string extension = strs[strs.size() - 1];

          if( file.rfind(ext) != std::string::npos )
          {
#if BOOST_FILESYSTEM_VERSION == 3
            std::string path = rel_path_so_far + (itr->path().filename()).string();
#else
            std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif
            std::string fullpath = rel_path_so_far + itr->path().string();
            relative_paths.push_back(path);
          }
        }
      }
    }

    void createTrainingDir(const std::string & training_dir)
    {
      if(!boost::filesystem::exists(training_dir))
        boost::filesystem::create_directory(training_dir);
    }

    void getArgvArgc(const std::string & commandline, char *** argv, int & argc)
    {
      enum
      {
        kMaxArgs = 64
      };

      argc = 0;
      *argv = new char *[kMaxArgs];
      (*argv)[argc++] = (char *) "program";

      char *p;
      p = strtok((char *) commandline.c_str(), " ");
      while(p && argc < kMaxArgs) {
        (*argv)[argc++] = p;
        p = strtok(0, " ");
      }
    }
  // fileutil namespace 
  }
  //od namespace
}

