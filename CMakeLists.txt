cmake_minimum_required(VERSION 2.8)
project(OpenDetection)

 # Set correct flags depending on the OS and compiler
if(WIN32) 
  if(MSVC) 
    set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} /SUBSYSTEM:WINDOWS /DNOMINMAX") 
  elseif(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mwindows -std=c++11 -Wall") 
  endif()
elseif(UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
endif()

# Initialize variables
set(OD_SOURCE_DIR ${OpenDetection_SOURCE_DIR})
set(OD_BINARY_DIR ${OpenDetection_BINARY_DIR})
set(OD_CMAKE_DIR ${OpenDetection_SOURCE_DIR}/cmake)
set(CMAKE_MODULE_PATH ${OD_CMAKE_DIR}/modules)
set(CMAKE_3RDPARTY_DIR ${OD_SOURCE_DIR}/3rdparty)

# Initialize versioning
include(${OD_CMAKE_DIR}/ODVersion.cmake)
set(OD_VERSION ${OD_MAJOR_VERSION}.${OD_MINOR_VERSION})
set(OD_VERSION_DETAILED ${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}.${OD_BUILD_VERSION})

# Set up macros and configurations
include(${OD_CMAKE_DIR}/ODTargets.cmake)
include(${OD_CMAKE_DIR}/ODDependency.cmake)
include(${OD_CMAKE_DIR}/ODMacros.cmake)
include(${OD_CMAKE_DIR}/ODDeb.cmake)

# Prepare configuration files
configure_file(${OD_CMAKE_DIR}/ODVersion.h.in ${OD_BINARY_DIR}/generated/od_version.h)
install(FILES ${OD_BINARY_DIR}/generated/od_version.h DESTINATION ${OD_INSTALL_INCLUDE_DIR})
configure_file(${OD_CMAKE_DIR}/ODConfig.cmake.in ${OD_BINARY_DIR}/FindOD.cmake)

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
  add_subdirectory(${OD_SOURCE_DIR}/${subdir})
endforeach(subdir)

# Install the FindPackage configuration file
get_property(OD_INSTALLED_LIBRARIES GLOBAL PROPERTY OD_INSTALLED_LIBRARIES)
install(FILES ${OD_BINARY_DIR}/FindOD.cmake DESTINATION ${OD_CMAKE_INSTALL_DIR})

# uninstall target
configure_file(
    "${OD_CMAKE_DIR}/ODUninstall.cmake.in"
    "${OD_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)