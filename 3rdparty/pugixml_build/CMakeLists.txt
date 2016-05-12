cmake_minimum_required(VERSION 2.8)
Project(pugixml CXX)

set(CMAKE_BUILD_TYPE Release) 
add_library(pugixml SHARED ${CMAKE_SOURCE_DIR}/3rdparty/pugixml/src/pugixml.cpp)
