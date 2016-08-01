cmake_minimum_required(VERSION 2.8)
project(OpenDetection)

# Initialize variables
set(OD_SOURCE_DIR ${OpenDetection_SOURCE_DIR})
set(OD_BINARY_DIR ${OpenDetection_BINARY_DIR})
set(OD_CMAKE_DIR ${OD_SOURCE_DIR}/cmake)
set(CMAKE_MODULE_PATH ${OD_CMAKE_DIR}/modules)
set(CMAKE_3RDPARTY_DIR ${OD_SOURCE_DIR}/3rdparty)
set(CMAKE_OD_GENERATED_DIR ${OD_BINARY_DIR}/generated)

# Initialize versioning
include(${OD_CMAKE_DIR}/Version.cmake)
set(OD_VERSION ${OD_MAJOR_VERSION}.${OD_MINOR_VERSION})
set(OD_VERSION_DETAILED ${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}.${OD_BUILD_VERSION})

# Set up macros and configurations
include(${OD_CMAKE_DIR}/Options.cmake)
include(${OD_CMAKE_DIR}/Targets.cmake)
include(${OD_CMAKE_DIR}/Dependency.cmake)
include(${OD_CMAKE_DIR}/Macros.cmake)
include(${OD_CMAKE_DIR}/Deb.cmake)
include(${OD_CMAKE_DIR}/Uninstall.cmake)

# Prepare configuration files
configure_file(${OD_CMAKE_DIR}/Version.h.in ${CMAKE_OD_GENERATED_DIR}/version.h)
install(FILES ${CMAKE_OD_GENERATED_DIR}/version.h DESTINATION ${OD_INSTALL_INCLUDE_DIR})

# Set modules
set(OD_MODULES_NAMES 3rdparty common gpu/common detectors gpu/detectors doc examples)

# Add modules
foreach(subdir ${OD_MODULES_NAMES})
  add_subdirectory(${OD_SOURCE_DIR}/${subdir})
endforeach(subdir)

# Install the FindPackage configuration file
get_property(OD_INSTALLED_LIBRARIES GLOBAL PROPERTY OD_INSTALLED_LIBRARIES)
configure_file(${OD_CMAKE_DIR}/Config.cmake.in ${OD_BINARY_DIR}/Config.cmake)
install(FILES ${OD_BINARY_DIR}/Config.cmake DESTINATION ${OD_CMAKE_INSTALL_DIR})
