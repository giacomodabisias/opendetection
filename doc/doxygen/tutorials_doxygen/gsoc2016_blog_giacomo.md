
GSoC 2016 Blog - Giacomo {#gsoc2016_blog_giacomo}
====
[TOC]

#Framework design and library maintenance#  {#gsoc2016_blog_giacomo1}
====

  - [Link to Proposal](https://docs.google.com/document/d/16Wyd0h5b9-7DaG7ZYJT30a2i096krviFUCcDYwg-jZc/edit?usp=sharing) - [Link to GSoC2016 Project Page](https://summerofcode.withgoogle.com/organizations/6007728078061568/#5675882488266752)
 
##About Me##

I finished my **joint master degree** in computer science and networking at Sant'Annas school of advanced studies and the university of Pisa in 2014 with a thesis on the static allocation of real-time OpenMP jobs on multicore machines. The master program was focused on parallel and high performance computing including OpenMP, MPI, Cuda and Tbb.

I started then working as a scholar on the Pelars project (Practice-based Experiential Learning Analytics Research And Support) at the Laboratory of Perceptual Robotics (PERCRO), which is part of the Institute of Communication, Information and Perception Technologies (TECIP) of the Scuola 
Superiore Sant’Anna, Pisa.

In November 2014 I started my PhD in Perceptual Robotics, researching Computer Vision for robotic applications. The first part of my research concerned object recognition algorithms, mainly developing real time solutions; I have worked with different approaches ranging from classical descriptor matching pipelines to machine learning techniques. Both 2D and 3D approaches have been investigated, using different platforms and sensors. I am also active in the research area of RGB-D cameras creating interfaces and testing new sensors.

I stayed as a visiting student for one month at **ETHZ** to investigating object recognition using **Random Forests**, which gave me some great insights on the use of machine learning tecniques in this filed. 

Code is mainly developed using **C**++, **Java**.
I am contributing to the different libraries like **PCL**, **OpenCV** and **libfreenect2**.

-[Link to Github](https://github.com/giacomodabisias)

##General Project Idea##

The concept of this gsoc project is to restructure the OpenDetection Library in order to make it more usable, scalable and documented. In a second phase there could also be the possibility to implement some new features in the library.

The implementation can be divided into several work tasks
- **Task 1** - fix dependencies and modular build : Restructure the Cmake files to obtain a clean and expandable structure. Remove as many dependencies as possible and set build options for the different OpenDetection modules based on the OpenCV/PCL dependencies. 
Depending on OpenCV/PCL version activate/deactivate modules of the library.
- **Task 2** - small tasks : Restructure the code by moving the different files into appropriate folders. Move 3rd party software to git sub modules or downloadable cmake content. Use more templetization and create precompiled versions for basic types. Enhance performances by optimizing the whole code structure. All these steps include the addition of a fixed coding style which can be derived or adopted by an existing one. 
- **Task 3** - moderate task - memory allocation : Restructure memory allocation using shared_pointers where possible to avoid dynamic memory allocation with new. 
- **Task 4** - moderate task - generalized viewer : Create a generalized viewer interface for 2D images and 3D point clouds based on VTK. 
- **Task 5** - moderate task - packaging and cross compilation : Fix the Cmake files in order to make the library cross platform buildable on Linux and Windows systems. Add header precompilation if possible and ProjectConfig.cmake files to to make the library usable by other projects. 
- **Task 6** - moderate - input enhancements : A lot of classes have the possibility to load data from file/path. Create template versions of them in order to let people also load data from vectors/arrays etc.. 
- **Task 8** - small - Documentation and examples : Continue the documentation process and add new examples for the new implemented structures/algorithms. 
- **Task 9** - small task - merge other gsoc16 contributions : Merge changes from other contribution of GSOC. Other projects will be using existing APIs. There might me need to make small changes to fit to the changes made in this project 
- **Task 10** -  moderate task - deb packaging : Create an automated way to generate a deb file for the library so it can be installed through debian packaging system.

##Fix 3rd party dependencies##

The library has some dependencies inserted as source code into the 3rdparty folder; these are pugixml and SiftGPU (and maybe others which will come later). There is also the dependency of svmlight, but this is not mandatory so it will be added as external dependency; if the library is present on the system all the depending libraries will be built.

In general its a bad idea to integrate external source code into a library since you can't have update versions which maybe fix bugs etc.. and you have to take care of licenses. To fix this issue I removed the two folders (**pugixml** and **SiftGPU**) and added the git repositories of these two libraries as submodules. This way, when a user is downloading the opendetection library, he will also download the lates version of the two libraries. One could argue that the API of the libraries could change, but it is possible to checkout a fixed version in order to avoid these issues.

- **SiftGPU** can just be added with the **add_subdirectory** cmake command which will execute the *CMakeLists.txt* in the library folder and export the produced targets (libsiftGPU.so in this case.
- **Pugixml** is a bit more tricky since it has no cmake file; it contains a Make file which produces a test executable but no library. The library can be easily built since we have just two include files and one .cpp file. To fix the issue I created a separate pugixml_build folder with a simple custom cmake which builds the library. This way we can just add that folder as add_subdirectory and directly export the newly compile **libpugixml.so** library.
- **Svmlight** will be inserted in the system with a find_package or something similar and a custom **WITH_SVMLIGHT** cmake flag. These steps will come shortly; we will just leave the svmlight binding.

##Refactoring file structure##

I started to move files in appropriate folders and to fix the "Main" *CmakeLists.txt* file. I renamed the ODconfig.h.in file to od_config.h and moved it into the cmake folder for now. Then I moved the opendetection.cpp source file away from the root (not nice to have source files in the root of a library) and added it as separate app in a new version folder which builds the od_version executable.

I fixed the **OD_ADD_EXAMPLE** macro adding the **INCLUDE** argument to include specific headers and used it in the *od_version* app. A separate cmake file has benn created to include all MACROS, but I have still to check if it is possible to add ordering between include cmake statements since there are some dependencies between the files. I separated the **OD_CMAKE_DIR** from **CMAKE_MODULE_PATH** in order to have a clear struture and to not mix up folders; the files in the cmake folder have also been renamed to have a common naming scheme.The next step will involve separating source and header files in appropriate folders and adding them in the cmake without explicitly naming them to avoid problems when renaming files in the library.

The next huge step consisted in refactoring common, detectors and example folders to have a common standard file structure. This is usually built having in each folder the following subflders/files:
- CMakeLists.txt : the usual cmake build file
- src : all the source files which can be subdivided into folders
- include : a folder hierarchy which resambles the src structure. This is very useful to install include files and to maintain clear include names
- impl : a floder containing the template files (.hpp usually).

This resembles also the structure used in the PCL library.

After refactoring the folders, all the involved CMakeFiles had to be restructured to adapt to the new structure. The first version still uses all file names explicitly, but it should be possible to automate the file detection process by using the cmake command *file(GLOB VAR PATTERN)* which finds all file in a gve folder which match a given pattern. This can be done also in a recursive manner. I will probably use this for some folders to be independent (partially) of file names, locations and number.

##Making examples optional##

Examples should not be built always, or at least it should be possible to not build them at all or partially. To do so I added the **WITH_EXAMPLES** option whichi enables the building of the examples. Then for each example I added a variable to trigger the building process of that examples:

option(image_hog_files_example "Build the hog image example" ON)
if(image_hog_files_example)
        OD_ADD_EXAMPLE(od_image_hog_files FILES od_image_hog_files.cpp
                        LINK_WITH od_common od_global_image_detector)
endif()

The option command adds an option in the cmake; the first parameter is the cmake option name (which for now is written in that way but it will probably change), the second is a description of the option, and the third is the default option value. I left it on on so that even unexperienced user can build some examples to test the library. The option is then followed by an if which checks the option vale and in case adds the example to the build process. I would like to change also the structure of the example folder by creating subfolders which then contain the different examples subdividing them by **type** . Each folder will then have its own *CMakeLists.txt* file which can be added by the parent cmake with the usual *add_subdirectory* command. This helps also since we can add automatically all the subfolders of the example folder to the build process without stating the names explicitly.