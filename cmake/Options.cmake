# Set correct flags depending on the OS and compiler
if(WIN32) 
  if(MSVC) 
    set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} /SUBSYSTEM:WINDOWS /DNOMINMAX") 
  elseif(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mwindows -std=c++11") 
  endif()
elseif(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release
RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# Optional parameters
option(WITH_WARNINGS "Add build warnings" OFF)
option(WITH_DOCUMENTATION "Build the OD documentation" ON)
option(WITH_GPU "Build GPU modules" ON)
option(WITH_EXAMPLES "Build examples" OFF)
option(WITH_SVMLIGHT "Build with svmlight support" ON)
option(WITH_STD_SHARED_PTR "use std::shared_ptr instead of boost::shared_ptr" OFF)
option(WITH_CAFFE "build caffe support" OFF)
option(WITH_GTKMM "build gtkmm support" OFF)

option(BUILD_GLOBAL_2D_DETECTION "build global 2D detection" ON)
option(BUILD_GLOBAL_3D_DETECTION "build global 3D detection" ON)
option(BUILD_LOCAL_2D_DETECTION "build local 2D detection" ON)
option(BUILD_MISC_DETECTION "build miscellaneous detection" ON)

if(WITH_WARNINGS)
  if(WIN32) 
    if(MSVC) 
      set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} /WALL") 
    elseif(CMAKE_COMPILER_IS_GNUCXX)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") 
    endif()
  elseif(UNIX)
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall")
  endif()
endif()

if(WITH_STD_SHARED_PTR)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_STD_SHARED_PTR")
endif()

if(WITH_GPU)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_GPU")
endif()

if(WITH_CAFFE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_CAFFE")
endif()

