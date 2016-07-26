cmake_minimum_required(VERSION 2.8)
project(OpenDetection)

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
include(${OD_CMAKE_DIR}/ODOptions.cmake)
include(${OD_CMAKE_DIR}/ODTargets.cmake)
include(${OD_CMAKE_DIR}/ODDependency.cmake)
include(${OD_CMAKE_DIR}/ODMacros.cmake)
include(${OD_CMAKE_DIR}/ODDeb.cmake)

# Prepare configuration files
configure_file(${OD_CMAKE_DIR}/ODVersion.h.in ${OD_BINARY_DIR}/generated/od_version.h)
install(FILES ${OD_BINARY_DIR}/generated/od_version.h DESTINATION ${OD_INSTALL_INCLUDE_DIR})

# Set modules
set(OD_MODULES_NAMES 3rdparty common "gpu/common" detectors "gpu/detectors" doc examples)
set(OD_MODULES_DIRS ${OD_MODULES_NAMES})

# Add modules
foreach(subdir ${OD_MODULES_DIRS})
  add_subdirectory(${OD_SOURCE_DIR}/${subdir})
endforeach(subdir)

# Install the FindPackage configuration file
get_property(OD_INSTALLED_LIBRARIES GLOBAL PROPERTY OD_INSTALLED_LIBRARIES)
configure_file(${OD_CMAKE_DIR}/ODConfig.cmake.in ${OD_BINARY_DIR}/ODConfig.cmake)
install(FILES ${OD_BINARY_DIR}/ODConfig.cmake DESTINATION ${OD_CMAKE_INSTALL_DIR})

# uninstall target
configure_file(
    "${OD_CMAKE_DIR}/ODUninstall.cmake.in"
    "${OD_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)