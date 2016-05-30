//
// Created by sarkar on 19.06.15.
//
#include "od/common/utils/utils.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/functional/hash.hpp>

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
      float norm=0;
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

  cv::Scalar getHashedColor(const std::string & name, int offset = 100)
  {
    boost::hash<std::string> string_hash;
    const int hashed = string_hash(name);

    return CV_RGB((hashed + offset) % 255, (hashed + 2*offset) % 255, (hashed + 3*offset) % 255);
  }

  cv::Mat makeCanvasMultiImage1(std::vector<cv::Mat> & input, int window_height, int rows)
  {
    const unsigned int input_size = input.size();
    const unsigned int n_rows  = rows > input_size ? input_size : rows;
    const unsigned int edge_thickness = 10;
    const unsigned int images_per_row = ceil(static_cast<float>(input_size) / static_cast<float>(n_rows));
    const unsigned int resize_height = floor(2.0 * ((floor(static_cast<float>(window_height - edge_thickness) / static_cast<float>(n_rows))) / 2.0)) - static_cast<float>(edge_thickness);
    unsigned int max_row_length = 0;

    std::vector<int> resize_width;

    for(size_t i = 0; i < input_size;){
      int this_row_len = 0;
      for(size_t k = 0; k < images_per_row; k++){
        float aspect_ratio = float(input[i].cols) / input[i].rows;
        const int temp = static_cast<int>(ceil(resize_height * aspect_ratio));
        resize_width.push_back(temp);
        this_row_len += temp;
        if(++i == input_size) 
          break;
      }
      const int tmp = this_row_len + edge_thickness * (images_per_row + 1);
      if(tmp > max_row_length){
        max_row_length = tmp;
      }
    }

    cv::Mat canvas_image(window_height, max_row_length, CV_8UC3, cv::Scalar(0, 0, 0));

    for(size_t k = 0, i = 0; i < n_rows; i++){
      int y = i * resize_height + (i + 1) * edge_thickness;
      int x_end = edge_thickness;
      cv::Size s;
      for(size_t j = 0; j < images_per_row && k < input_size; k++, j++){
        cv::Rect roi(x_end, y, resize_width[k], resize_height);
        s = canvas_image(roi).size();
        // change the number of channels to three
        cv::Mat target_roi(s, CV_8UC3);
        if(input[k].channels() != canvas_image.channels()){
          if(input[k].channels() == 1){
            cv::cvtColor(input[k], target_roi, CV_GRAY2BGR);
          }
        }
        cv::resize(target_roi, target_roi, s);
        if(target_roi.type() != canvas_image.type()){
          target_roi.convertTo(target_roi, canvas_image.type());
        }
        target_roi.copyTo(canvas_image(roi));
        x_end += resize_width[k] + edge_thickness;
      }
    }
    return canvas_image;
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
            return "";
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
              for (int exti = 0; exti < exts.size(); exti ++)
                if(file.rfind(exts[exti]) != std::string::npos)
                { flagfound = true; break; }

              if( flagfound == true )
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

