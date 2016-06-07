
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

##Fix 3rd party dependencies 12/05/15##

The library has some dependencies inserted as source code into the 3rdparty folder; these are pugixml and SiftGPU (and maybe others which will come later). There is also the dependency of svmlight, but this is not mandatory so it will be added as external dependency; if the library is present on the system all the depending libraries will be built.

In general its a bad idea to integrate external source code into a library since you can't have update versions which maybe fix bugs etc.. and you have to take care of licenses. To fix this issue I removed the two folders (**pugixml** and **SiftGPU**) and added the git repositories of these two libraries as submodules. This way, when a user is downloading the opendetection library, he will also download the lates version of the two libraries. One could argue that the API of the libraries could change, but it is possible to checkout a fixed version in order to avoid these issues.

- **SiftGPU** can just be added with the **add_subdirectory** cmake command which will execute the *CMakeLists.txt* in the library folder and export the produced targets (libsiftGPU.so in this case.
- **Pugixml** is a bit more tricky since it has no cmake file; it contains a Make file which produces a test executable but no library. The library can be easily built since we have just two include files and one .cpp file. To fix the issue I created a separate pugixml_build folder with a simple custom cmake which builds the library. This way we can just add that folder as add_subdirectory and directly export the newly compile **libpugixml.so** library.
- **Svmlight** will be inserted in the system with a find_package or something similar and a custom **WITH_SVMLIGHT** cmake flag. These steps will come shortly; we will just leave the svmlight binding.

##Refactoring file structure 13/05/15##

I started to move files in appropriate folders and to fix the "Main" *CmakeLists.txt* file. I renamed the ODconfig.h.in file to od_config.h and moved it into the cmake folder for now. Then I moved the opendetection.cpp source file away from the root (not nice to have source files in the root of a library) and added it as separate app in a new version folder which builds the od_version executable.

I fixed the **OD_ADD_EXAMPLE** macro adding the **INCLUDE** argument to include specific headers and used it in the *od_version* app. A separate cmake file has benn created to include all MACROS, but I have still to check if it is possible to add ordering between include cmake statements since there are some dependencies between the files. I separated the **OD_CMAKE_DIR** from **CMAKE_MODULE_PATH** in order to have a clear struture and to not mix up folders; the files in the cmake folder have also been renamed to have a common naming scheme.The next step will involve separating source and header files in appropriate folders and adding them in the cmake without explicitly naming them to avoid problems when renaming files in the library.

The next huge step consisted in refactoring common, detectors and example folders to have a common standard file structure. This is usually built having in each folder the following subflders/files:
- CMakeLists.txt : the usual cmake build file
- src : all the source files which can be subdivided into folders
- include : a folder hierarchy which resambles the src structure. This is very useful to install include files and to maintain clear include names
- impl : a floder containing the template files (.hpp usually).

This resembles also the structure used in the PCL library.

After refactoring the folders, all the involved CMakeFiles had to be restructured to adapt to the new structure. The first version still uses all file names explicitly, but it should be possible to automate the file detection process by using the cmake command *file(GLOB VAR PATTERN)* which finds all file in a gve folder which match a given pattern. This can be done also in a recursive manner. I will probably use this for some folders to be independent (partially) of file names, locations and number.

##Making examples optional 17/05/15##

Examples should not be built always, or at least it should be possible to not build them at all or partially. To do so I added the **WITH_EXAMPLES** option whichi enables the building of the examples. Then for each example I added a variable to trigger the building process of that examples:

@code
option(image_hog_files_example "Build the hog image example" ON)
if(image_hog_files_example)
        OD_ADD_EXAMPLE(od_image_hog_files FILES od_image_hog_files.cpp
                        LINK_WITH od_common od_global_image_detector)
endif()
@endcode

The option command adds an option in the cmake; the first parameter is the cmake option name (which for now is written in that way but it will probably change), the second is a description of the option, and the third is the default option value. I left it on on so that even unexperienced user can build some examples to test the library. The option is then followed by an if which checks the option vale and in case adds the example to the build process. I would like to change also the structure of the example folder by creating subfolders which then contain the different examples subdividing them by **type** . Each folder will then have its own *CMakeLists.txt* file which can be added by the parent cmake with the usual *add_subdirectory* command. This helps also since we can add automatically all the subfolders of the example folder to the build process without stating the names explicitly.

##Include structure 18/05/15 ##

While continuing to restructure the library I came across a decision which can be solved in different ways but for which I still don't have the best solution. Lets assume we want to include a global 3D detector for example, we would use *ODCADDetector3DGlobal.h* . We could have in our file 

@code
#include <ODCADDetector3DGlobal.h> 
@endcode

and the compiler would need the include folder where the file is located to compile. This means that the file should be localted directly in the upper include folder, but this is not our case since the structure is like *detectors/global3D/detection/* so it would be better to have

@code
#include <detectors/global3D/detection/ODCADDetector3DGlobal.h> 
@endcode

and include the upper level folder. To use the first solution we would need to include *detectors/global3D/detection/* but this is not really appealing since it would mess with the whole include structure. For now I opted for the second solution, but the include structure could be changed at any time. This way the include file structure resables exactly the source file structure:

- common
	* bindings
	* pipeline
	* utils
- detectors
	* global2D
		- detection
		- training
	* global3D
		- detection
		- training
	* local2D
		- detection
		- training
	* misc
		- detection

The new structure compiles fines except for three examples which have a linker bug (undefined reference to `vtable for od::g3d::ODCADDetectTrainer3DGlobal'
) which I am trying to resolve. I fixed a bit the source code of all the examples removing unnecessary includes, fixing namespaces, maintaining a common interface and avoiding dynamic memory allocation where possible. The next step after fixing the linker bug will be to fix the install paths for includes and libs. 

##Include structure and install target 23/05/15##

I fixed the linking error; it came from a wrong variable name in a cmake file which specified some source files which were not compiled and so the linking error. I split simple_ransac_detection in src and include and reinserted it in local2D detectors since it is used only there. The source files are just added to the local2D library and compiled together.

Next I moved to fix the install target in the cmake. In our case, since we have already a clean structure of the include folders which we want to maintain we can directly copy the include folders of each module (detectors and common) into the include folder specified by *CMAKE_INSTALL_PREFIX* . We just need to create an upper folder called od-$VERSION to avoid conflicts with different library versions on the same machine. The clean include structure allows also to remove the explicit single include files from the install targets and allows to use directly the cmake command

@code
install(DIRECTORY ${DETECTORS_INCLUDE_DIR}/od DESTINATION ${OD_INSTALL_INCLUDE_DIR})
@endcode

with the DIRECTORY keyword to copy the whole folder. Also I removed the include files from the 

@code
OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${SOURCES})
@endcode

since you don't have to compile headers files.It is enough to specify the include folders to find the includes at compile time. It could be possible to add header precompilation, but I believe this would be an advanced step which could be implemented at the end. I still need to check if this way of installing includes using the *DIRECTORY* keyword without specifying the single files is the clearest way.


##Code refactoring 01/06/15##

Before modifying and digging into the code I wanted to have a common coding style in all files, fixing also some common coding mistakes when found. Another decision was to use C++11 features whenever possible since it should be considered a basic standard, having already C++17 code online. I started by fixing the "common module"; here I did the following:

- I removed the ToString method since in c++11 it is present as *to_string()*; 
- I removed global variables from utils since they where just used in one function.
- I removed the *FileUtils* class which was a class with only static functions. This is equivalent to the new structure which is a namespace with defined functions which is more readable and less prone to errors.
- I removed the time.h headers everywhere in order to use chrono from the std library.
- I added constness modifiers and references for passed arguments whereever possible to avoid useless parameter copyes.
- I modified the frame generator template to have as parameter generic clouds and not only pointrgba point clouds

After this I started to fix the detectors which I am still working on. The first class which has bee updated is the *ODHOGDetector* which had also a lot of naming issues since it used a different naming style. 
There are stll some function which I dislike, for example the parsing methods. They are mainly based on creating a fake argc,argv couple of variables and the parsing them. This has to be removed in favor of a better and newer parsing library as the one included in boost. This will be one of the first steps which I will do as soon as I finished the basic refactoring step.

##Code refactoring 2 07/06/15##

The first code refactoring phase terminated today, since I finished to review a bit all files; the main steps have been:

- moving around code in order to have implementations in the cpp files to reduce multiple compilation time, and declarations in .h files. 
- added some minor c++11 features like *ranged-loops* instead of iterators which are hard to read, more standard function like *to_string()* and *itos()* where possible
- removed standard namespace and opencv namespace to avoid name clashing
- refactored the whole simple ransac file folder which was outside of the namespace
- used a common coding style through all files which is fundamental
- I moved the **ODDetectorMultiAlgo.h** and **ODDetectorMultiAlgo.cpp** files into ODDetectorMultiAlgo.hpp since I made the classes template to avoid having the explicit **Pcl::PointXYZRGBA** type since it was using ODCADDetector3DGlobal.hpp which already defined template classes. The new file is situated in the impl folder in detectors to maintain the usual include structure. Now, since the default vale has been removed for the template classes, the type has to be specified when the class is instantiated which can been seen in the examples.

Missing parts and next steps:

- I have to check that compilation works fine also with svmlight which was disabled till now
- Use shared pointer where normal pointer are used. We have to decide if we want to use boost or stds shared pointers since pcl is using the first version. The second would be better so we would have everything which is possible using std and then when pcl moves toward std shared pointer we move also the rest of the code to that.
- Remove where possible the argc,argv parsing methods using standard ones.