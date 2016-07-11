
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

##Shared Pointers 10/06/15##

The following step took quite a bit in order to have everything work well. Until now the library was using normal C++ pointer created with new and then deleted when not used anymore. This can lead to bad habits of forgetting to delete pointers, causing memory leaks. To avoid this its always good to use safe pointers which get deallocated as soon as they are not referenced my anyone anymore. Safe pointers are called **shared_ptr** in C++ and were intially implemented in the *Boost* library; then, after C++11, they where also inserted in the standard in the *std::* namespace. Substituting all pointers n the library was quite a complex task for different reasons:

- Opendetection uses quite a lot the *Pcl* library. This library is still using the **boost::shared_ptr** implementation instead of the **std::shared_ptr** implementation. The two **shared_ptr** are not compatible and so a big issue arises. Should we use the first or second implementation? We should use the second solution since it relies on the standard, but we can't since *Pcl* still uses the first implementation. While *Pcl* moves to the *std::* solution we need to use the *boost* namespace, so in order to have the library compatible with both implementations, beeing able to switch from one to the other, I added a simple header file which switches between the two shared pointer types. This way as soon as *Pcl* moves to the *std::* we can also switch by just setting a variable in the cmake. The variable is called **WITH_BOOST_SHARED_PTR** ad is set to on by default so that we use *boost::shared_ptr* for now. 

- Substituting a pointer with a shared pointer requirest also to use another set of functions for the creation and the casting. to create a **shared_ptr** you either pass the pointer to the constructor or use the **make_shared** template passing to the function the constructor arguments and as template argument the pointer type. Also static and dynamic casting have specific template functions which are **dynamic_pointer_cast** and **static_pointer_casto**.

- It is important to remove all delete which are around the code since **shared_ptr** automatically destroy the pointed objects in the destructor if not referenced anymore.

While restructuring the code I also fixed again some coding style issues, but I still have to go through the code a few more times in order to finish fixing everything. The next step is as previously mentioned, to check the compilation with the svmlight option.

##Cmake improvments 20/06/15##

I created a **FindOD.cmake** which can be used by other libraries to find the include directories path, the library path and the library names. The variables which get set are:

- OD_INCLUDE_DIRS 
- OD_LIBRARY_PATH 
- OD_LIBRARIES 

There variables can be used in any project by using:

@code
find_package(OD REQUIRED)
include_directories(${OD_INCLUDE_DIRS})
link_directories(${OD_LIBRARY_PATH})
target_link_libraries(example_target ${OD_LIBRARIES})
@endcode

The **FindOD.cmake** is generated automatically starting from the **${CMAKE_INSTALL_PREFIX}** variable so there is no absolute path and it is installed in **${CMAKE_INSTALL_PREFIX}/lib/cmake/** .

I added then an **uninstall** target to delete all the installed files if necessary. This custom target is standard and can be found easily on the internet.

Another important missing thing was the creation of a .deb package which can be useful to install the library on other systems. This package has been created using the **Cpack** utility provided by cmake. By just setting some basic variables it is possible to create e complete .deb file which can then be installed using the usual *dpkg -i package.deb* command.

All files in the library have been renamed according to a common scheme which consists in a Prefix "OD" followed by the first charachter of the file name uppercase.

##Templates and Windows compilation 27/06/15##

To improve compilation time of projects using the **OpenDetection** library I decided to precompile all the templates for the most common types. In this case almost all templates depend on *pcl* point types. The most commonly used point type for which I precompiled the library are:

- *pcl::PointXYZ*
- *pcl::PointXYZRGB*
- *pcl::PointXYZRGBA*

It is important to create for each .hpp header file containing the templates a .cpp file in which templates are instantiated with the different types. Explicit template instantiation is done by just writing *template* followed by the template declaration with the instantiated type.
The most important part is to insert in the header file the same declaration with the *extern* keyword at the beginning. This informs the compiler that the defined symbol is already present in another compilation unit and that it can be found at linking time. Without this the compiler will just reinstantiate the templates and recompile them since it has no knowledge about templates instantiated and compiled in other compilation units.

I then fixed all the linking in the whole library since there where some useless linked libraries in order to speed up even more the compilation. 

After that I tested compilation of the library under Windows, but without success. There have been several issues which are ut of the Open Deteciton library. I tested compilation using **Visual Studio 2015** and **MinGW**. Pcl has been installed by using the prebuilt version while opencv has to be built manually since *opencv_contrib* is necessary while not present in the prebuilt binaries. Building opencv3 with visual studio and cuda is not possible at the moment since cuda 7.5 is not compatible with visual studio 2015, so version 8 is necessary which will be available soon. MinGW is also not usable since compilation of opencv3 with cuda is disabled as flagged as incompatible by opencv.

##Face detection 31/06/15##

To test the new cmake and code structure I **implemented face detection** (not recognition which is already available). Face detection is implemented in *OpenCV* both in **CPU** mode and in **GPU** mode (using **CUDA**). 

I created an interface which is using the *ODDetector2D* interface to use both **CPU** and **GPU** mode based on a parameter passed in the constructor. **GPU** mode is only available if the library was built with **GPU** support (using the *WITH_GPU* flag in the cmake file). 

To test the new detector I created two examples; the first one is *od_face_detecor_cam.cpp* which uses the first found webcam to detect faces; the second example is called *od_face_detector_files.cpp* which uses a path to a file folder to execute the face detection algorithm over all files present in that folder. 

I noticed that it is necessary to pass to the *ODFrameGenerator* a file expression like /home/test/images/\*.jpg. I don't really like this method since it is quite restrictive and not intuitive to call. I will add also a function using **boost filesystem** to iterate over all files which are OpenCV supported images, using as input a path name.
I added in the cmake the two new examples with a custom option and tested them successfully.

The next step will be to add the functionality previously described to the frame generator; I will also add the possibility to use standard containers as input sources in roder to be able to use arrays, vectors and lists for example.

##Face detection 2 06/06/15##

After talking with Kripa I found out that the CPU cascade detector (the detector used to detect faces) was already rpesent n the library; so only the gpu version had to be added. This implies to create in the library a new gpu folder containing all gpu modules which should be built only if support is present.

To do this I created a separate gpu folder which contains for now a detector folder with the same structure as the external one. Everything defined here should be in the **od::gpu::** namespace and each class should have the same name as the cpu version class. It is also a good habit to first create a common interface for the **CPU** and **GPU** version in order to be able to dinamically allocate objects of both types.

To grasp better the meaning I implemented the **cascade detector** as follows:

Both detectors derive already from the same class so there is no need for an additional interface

@code
class ODCascadeDetector : public ODDetector2D
@endcode

Each class has a different detector member:

GPU
@code
cv::Ptr<cv::cuda::CascadeClassifier> haar_cascade_;
@endcode

CPU
@code
shared_ptr<cv::CascadeClassifier> haar_cascade_;
@endcode

This way we can do the following when using the detectors:

@code

	boost::shared_ptr<od::ODDetector2D> detector;

	if(gpu)
	{
		detector = boost::make_shared<od::gpu::g2d::ODCascadeDetector>(argv[1]);
	}else{
		detector = boost::make_shared<od::g2d::ODCascadeDetector>(argv[1]);
	}

@endcode

There were three possible solutions to how to impement gpu in the library:

- Add in the constructor a flag to decide the **CPU** / **GPU** mode. This mixes up gpu and cpu code in the same class and changing anything becomes more and more difficult

- Create two separate classes with dfferent names,  one for each type. This fixes the previous issue but moving from one version to the other can be hard

- Create a common interface and have the same class imlemented in different namespaces. Clean solution which has been adopted.

I removed the previous facedetector classes which I had added and used the new ones in the examples which I had previoulsy created. I also used the new viewer in all examples and fixed the build of the different examples to be independent and clean in the *CMake*.