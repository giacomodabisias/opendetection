cmake_minimum_required(VERSION 2.8)
project(OpenDetection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Initialize variables
set(OD_SOURCE_DIR ${OpenDetection_SOURCE_DIR})
set(OD_BINARY_DIR ${OpenDetection_BINARY_DIR})
set(OD_CMAKE_DIR "${OpenDetection_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${OD_CMAKE_DIR}/modules)
set(CMAKE_3RDPARTY_DIR ${OD_SOURCE_DIR}/3rdparty)


# Initialize versioning
include(${OD_CMAKE_DIR}/od_version.cmake)
set(OD_VERSION "${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}")
set(OD_VERSION_DETAILED "${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}.${OD_BUILD_VERSION}")
configure_file("${OD_CMAKE_DIR}/od_config.h.in" "${OD_BINARY_DIR}/generated/od_config.h")

# Set up macros and configurations
include(${OD_CMAKE_DIR}/od_targets.cmake)
include(${OD_CMAKE_DIR}/od_dependency.cmake)
include(${OD_CMAKE_DIR}/od_macros.cmake)

# Optional parameters
option(WITH_DOCUMENTATION "Build the OD documentation" ON)
option(WITH_GPU "Build GPU components of the project" ON)
option(WITH_EXAMPLES "Build examples" OFF)
option(WITH_BOOST_SHARED_PTR "use boost::shared_ptr instead of std::shared_ptr" ON)

if(WITH_BOOST_SHARED_PTR)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_BOOST_SHARED_PTR")
endif()

# Set modules
set(OD_MODULES_NAMES 3rdparty common detectors doc examples)
set(OD_MODULES_DIRS ${OD_MODULES_NAMES})

# Add modules
foreach(subdir ${OD_MODULES_DIRS})
  add_subdirectory("${OD_SOURCE_DIR}/${subdir}")
endforeach(subdir)

