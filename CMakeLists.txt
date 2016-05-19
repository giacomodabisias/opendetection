cmake_minimum_required(VERSION 2.8)
project(OpenDetection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")

# Initialize variables
set(OD_SOURCE_DIR ${OpenDetection_SOURCE_DIR})
set(OD_BINARY_DIR ${OpenDetection_BINARY_DIR})
set(OD_CMAKE_DIR "${OpenDetection_SOURCE_DIR}/cmake" )
set(CMAKE_MODULE_PATH ${OD_CMAKE_DIR}/modules)

# Initialize versioning
include(${OD_CMAKE_DIR}/od_version.cmake)
set(OD_VERSION "${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}")
set(OD_VERSION_DETAILED "${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}.${OD_BUILD_VERSION}")
configure_file ("${OD_CMAKE_DIR}/od_config.h.in" "${OD_BINARY_DIR}/generated/od_config.h" )

# Set up macros and configurations
include(${OD_CMAKE_DIR}/od_targets.cmake)
include(${OD_CMAKE_DIR}/od_dependency.cmake)
include(${OD_CMAKE_DIR}/od_macros.cmake)

# Optional parameters
option(WITH_DOCUMENTATION "Build the OD documentation" ON)
option(WITH_GPU "Build GPU components of the project" ON)
option(WITH_SVMLIGHT "Build SVM Light" OFF)
option(WITH_EXAMPLES "Build examples" OFF)

# Set modules
set(OD_MODULES_NAMES 3rdparty common doc detectors examples)
set(OD_MODULES_DIRS ${OD_MODULES_NAMES})

# Add modules
foreach(subdir ${OD_MODULES_DIRS})
  add_subdirectory("${OD_SOURCE_DIR}/${subdir}")
endforeach(subdir)

