
Installation Instructions {#installation_instruction} 
=========================

 [TOC]
 
The compilation and installation of OD is fairly simple. You first have to install its dependencies before compiling. Folllowing are the dependencies with their specific settings:


Dependencies {#installation_instruction1}
============


##OpenCV 3.0## {#installation_instruction2}

OpenCV 3.0 is to be compiled with the modules xfeatures2d (for features like SIFT) and CUDA for GPU features if needed.
    
- **Source:** http://opencv.org/downloads.html or https://github.com/Itseez/opencv              
    
- **Required settings:**

    - *OpenCV contrib* - for xfeatures2d module handing SIFT/SURF features
        
        Detailed instructions with source are provided here: https://github.com/itseez/opencv_contrib . You need to download this seperate repository before compiling OpenCV.
                
        *CMAKE options*: OPENCV_EXTRA_MODULES_PATH=\<path_to_opencv_contrib\>/modules
                                              
    - *OpenCV CUDA module* - for GPU enabled feature detectors and matcher.
             
        *CMAKE options*: WITH_CUDA=ON       


##PCL 1.8 or above {#installation_instruction3}   
    
- **Source:** https://github.com/PointCloudLibrary/pcl or https://github.com/PointCloudLibrary/pcl/releases

- **Required settings:**

  * *3d_rec_framework* - for ESF, ESF etc recognition pipeline. To build with this setting you need to install OpenNI as well, which is the mandatory dependency for this app. Please refer to the PCL website for the version information. 
  
      *Additional CMAKE options:* BUILD_apps=ON, BUILD_apps_3d_rec_framework=ON
  

##SiftGPU {#installation_instruction4}
SiftGPU comes as a submodule in the 3rdparty folder. It can be enabled or disabled using the WITH_GPU option.

##PugiXML {#installation_instruction5}
PugiXML comes as a folder in the 3rdparty folder and is mandatory. 

##SVMLight{#installation_instruction6}
SVMlight comes as a submodule in the 3rdparty folder. It can be enabled or disabled using the WITH_SVMLIGHT option.


Installing Open Detection {#installation_instruction7}
====

With the above dependencies installed, OD should compile without any problem. Download the source from https://github.com/krips89/opendetection and compile it with default cmake options. The code while platform independent, is only tested and run in Linux machine. Instructions for the usage for linux are provided below: 

##Instructions: {#installation_instruction8}
Compile out of source using cmake+your favorite compiler. For example:

Download the code: 
@code{.bash}
cd <path_to_desired_download_location>
git clone --recursive https://github.com/krips89/opendetection.git
@endcode

configure with CMake and compile
There are several options in the cmake file which can be set to build examples, change installation path and add gpu support.
@code{.bash}
cd <path_to_source>
mkdir build; 
cd build
cmake ..
make -j
make install
@endcode

There are also other targets:
* uninstall to remove the library 
* package to build a .deb packge




  