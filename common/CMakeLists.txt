set(SUBSYS_NAME common)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The core module of OpenDetection having the pipeline logic")
set(SUBSYS_DEPS ${OpenCV_LIBS} ${PCL_LIBRARIES} GLEW)

set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(COMMON_IMPL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/impl)

set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
set(COMMON_IMPL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/impl PARENT_SCOPE)

if(WITH_GPU)
  set(SUBSYS_DEPS ${SUBSYS_DEPS} siftgpu)
endif()

set(SUBSYS_DEPS ${SUBSYS_DEPS} svmlight)

set(SOURCES
        "src/pipeline/ODObjectDetector.cpp"
        "src/pipeline/ODScene.cpp"
        "src/pipeline/ODDetection.cpp"
        "src/utils/ODUtils.cpp"
        "src/utils/ODFeatureDetector2D.cpp"
        "src/bindings/ODSvmlight.cpp"
        "src/utils/ODViewer.cpp"
    )

include_directories(${COMMON_INCLUDE_DIR})
include_directories(${COMMON_IMPL_DIR})
include_directories(${CMAKE_3RDPARTY_DIR}/svmlightlib/)
include_directories(${CMAKE_3RDPARTY_DIR}/SiftGPU/src/SiftGPU/)
include_directories(${PCL_INCLUDE_DIRS})

OD_ADD_LIBRARY(${SUBSYS_NAME} SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})
install(DIRECTORY ${COMMON_INCLUDE_DIR}/od DESTINATION ${OD_INSTALL_INCLUDE_DIR} COMPONENT ${LIB_NAME})
install(DIRECTORY ${COMMON_IMPL_DIR}/od DESTINATION ${OD_INSTALL_INCLUDE_DIR} COMPONENT ${LIB_NAME})

