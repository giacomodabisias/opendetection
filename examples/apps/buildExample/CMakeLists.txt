# Example on how to build a project using the OpenDetection Library

cmake_minimum_required(VERSION 2.8)
project(viewer)

find_package(OD REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OD_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OD_LIBRARY_PATH})

add_executable(viewer VisualizationTest.cpp)
target_link_libraries(viewer ${OD_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

