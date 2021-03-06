#pragma once
#include "od/common/pipeline/Detection.h"
#include "od/common/pipeline/Scene.h"
#include <iostream>
#include <opencv2/videoio.hpp>
#include <pcl/apps/3d_rec_framework/tools/openni_frame_source.h>
#include <boost/make_shared.hpp>

namespace od
{
  enum GeneratorType
  {
    GENERATOR_TYPE_FILE_LIST, GENERATOR_TYPE_DEVICE, GENERATOR_TYPE_CONTAINER
  };


  /** \brief The FrameGenerator class for capturing and reading Scenes conveniently.
   * Templated with two parameters - SceneType identifying a Scene class, and TYPE identifying the type of input. After the instantiation with a correct TYPE, use the function
   * getNextFrame() to get an instance of next scene of SceneType. getNextFrame() returns valid scenes until all scenes matched are exhausted - the time  when 'isValid()' is false.
   *
   * \tparam SceneT One of the Scene classes - SceneImage or ScenePointCloud
   * \tparam TYPE TYPE can be GENERATOR_TYPE_FILE_LIST which means you provide the list of scene files to be returned by the FrameGenerator in the constructor (for eg. \/home/username/pics/\*.jpg)
   * Or it can be GENERATOR_TYPE_DEVICE which picks up the webcam or the kinect based on the SceneType.
   * \author Kripasindhu Sarkar
   *
   */
  template<typename SceneT, GeneratorType TYPE>
  class FrameGenerator
  {
  public:

    FrameGenerator(const std::string & input = std::string(""));
    FrameGenerator(int input = 0){}

    shared_ptr<SceneT> getNextFrame();

    bool isValid();

  protected:

    std::vector<std::string> file_list_;
    std::string video_read_path_;
    unsigned int curr_image_;
    cv::VideoCapture inputCapture_;
    bool exhausted_;

  };


  template<typename SceneT>
  class FrameGenerator<SceneT, GENERATOR_TYPE_FILE_LIST>
  {
  public:
    FrameGenerator(const std::string & input = std::string(""))
    {
      file_list_ = od_glob(input);
      curr_image_ = -1;
      exhausted_ = false;
    }

    shared_ptr<SceneT> getNextFrame()
    {
      if(exhausted_)
      {
        cout << "Files Exhausted!";
        return nullptr;
      }

      curr_image_++;
      if(curr_image_ == file_list_.size() - 1)
        exhausted_ = true;

      cout << "Frame: " << file_list_[curr_image_] << endl;
#ifndef WITH_BOOST_SHARED_PTR
      return shared_ptr<SceneT>(new SceneT(file_list_[curr_image_]));
#else
      return make_shared<SceneT>(file_list_[curr_image_]);
#endif
    }

    bool isValid() 
    {
      return !exhausted_;
    }

    std::string currentFile()
    {
      return file_list_[curr_image_];
    }

  private:

    std::vector<std::string> file_list_;
    bool exhausted_;
    int curr_image_;

  };

/*
  template<typename SceneT, template<class, class> class TContainer, class TObject>
  class FrameGenerator<SceneT, GENERATOR_TYPE_CONTAINER>
  {
  public:

    explicit FrameGenerator(TContainer<TObject*, std::allocator<TObject*>> & container) :container_(container)
    {
      curr_image_ = -1;
      exhausted_ = false;
    }

    shared_ptr<SceneT> getNextFrame()
    {
      if(exhausted_)
      {
        cout << "Files Exhausted!";
        return nullptr;
      }

      curr_image_++;
      if(curr_image_ == file_list_.size() - 1)
        exhausted_ = true;

    }

    bool isValid() 
    {
      return !exhausted_;
    }

  private:

    TContainer<TObject*, std::allocator<TObject*>> & container_;
    bool exhausted_;
    int curr_image_;

  };
*/

  
  template<>
  class FrameGenerator<SceneImage, GENERATOR_TYPE_DEVICE>
  {
  public:

    FrameGenerator(const std::string & input = std::string(""))
    {
      std::cout << "Opening :" << input << std::endl;
      input_capture_.open(input);
      if(!input_capture_.isOpened()) 
        {std::cout << "FATAL: Cannot open video capture!" << std::endl;}
    }

    FrameGenerator(int input = 0)
    {
      std::cout << "Opening cam :" << input << std::endl;
      input_capture_.open(input);
      if(!input_capture_.isOpened()) 
        {std::cout << "FATAL: Cannot open video capture!" << std::endl;}
    }

    shared_ptr<SceneImage> getNextFrame()
    {
      cv::Mat frame; 
      input_capture_.read(frame);
      return make_shared<SceneImage>(frame);
    }

    bool isValid() 
    {
      return input_capture_.isOpened();
    }

    cv::VideoCapture input_capture_;
  };

  /*
  template<>
  class FrameGenerator<ScenePointCloud<pcl::PointXYZRGBA> , GENERATOR_TYPE_DEVICE>
  {
  public:


    FrameGenerator(std::string input = "")
    {

      camera_ = new OpenNIFrameSource::OpenNIFrameSource(input);

      keyboard_cb = boost::bind (&OpenNIFrameSource::OpenNIFrameSource::onKeyboardEvent, camera_, _1);
    }

    boost::function<void(pcl::visualization::KeyboardEvent const &)> const &getKeyboard_cb() const
    {
      return keyboard_cb;
    }

    ScenePointCloud<pcl::PointXYZRGBA> * getNextFrame()
    {
      OpenNIFrameSource::PointCloudPtr frame = camera_->snap();

      return new ScenePointCloud<pcl::PointXYZRGBA>(frame);
    }
    bool isValid() {return camera_->isActive();}

    OpenNIFrameSource::OpenNIFrameSource *camera_;
    boost::function<void (const pcl::visualization::KeyboardEvent&)> keyboard_cb;
  };
*/


  template<typename PointT>
  class FrameGenerator<ScenePointCloud<PointT>, GENERATOR_TYPE_DEVICE>
  {
  public:

    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    /* A simple class for capturing data from an OpenNI camera */
    FrameGenerator(const std::string & input = std::string("")) : grabber_(input), most_recent_frame_(), frame_counter_(0), active_(true)
    {
      boost::function<void(const PointCloudConstPtr&)> frame_cb = boost::bind(&FrameGenerator<ScenePointCloud<PointT> , GENERATOR_TYPE_DEVICE>::onNewFrame, this, _1);
      grabber_.registerCallback(frame_cb);
      grabber_.start ();
      boost::this_thread::sleep(boost::posix_time::seconds(5));
    }

    ~FrameGenerator()
    {
      grabber_.stop ();
    }

    shared_ptr<ScenePointCloud<PointT>> getNextFrame()
    {
      OpenNIFrameSource::PointCloudPtr frame = snap();

      return make_shared<ScenePointCloud<PointT>>(frame);
    }

    bool isValid() 
    {
      return isActive();
    }

    const PointCloudPtr snap ()
    {
      return most_recent_frame_;
    }

    bool isActive ()
    {
      return active_;
    }
    void onKeyboardEvent(const pcl::visualization::KeyboardEvent & event)
    {
      // When the spacebar is pressed, trigger a frame capture
      mutex_.lock ();
      if (event.keyDown () && event.getKeySym () == "e")
      {
        active_ = false;
      }
      mutex_.unlock ();
    }

  protected:

    void onNewFrame(const PointCloudConstPtr & cloud)
    {
      mutex_.lock ();
      ++frame_counter_;
      most_recent_frame_ = make_shared<PointCloud>(*cloud); // Make a copy of the frame
      mutex_.unlock ();
    }

    pcl::OpenNIGrabber grabber_;
    PointCloudPtr most_recent_frame_;
    unsigned int frame_counter_;
    boost::mutex mutex_;
    bool active_;
  };
}
