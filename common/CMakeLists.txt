set(SUBSYS_NAME common)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The core module of OpenDetection having the pipeline logic")
set(SUBSYS_DEPS ${OpenCV_LIBS} ${PCL_LIBRARIES} GLEW)

set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(COMMON_IMPL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/impl)

set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
set(COMMON_IMPL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/impl PARENT_SCOPE)

if(WITH_GPU)
  set(SUBSYS_DEPS ${SUBSYS_DEPS} od_gpu_common)
  include_directories(${GPU_COMMON_INCLUDE_DIR})
endif()

set(SOURCES
        "src/pipeline/ObjectDetector.cpp"
        "src/pipeline/Scene.cpp"
        "src/pipeline/Detection.cpp"
        "src/utils/Utils.cpp"
        "src/utils/FeatureDetector2D.cpp"
        "src/utils/Viewer.cpp"
        "src/utils/FeatureDetector.cpp"
        "src/utils/FeatureDetectorInterface.cpp"
    )

if(WITH_SVMLIGHT)
    set(SOURCES ${SOURCES} "src/bindings/Svmlight.cpp")
    include_directories(${CMAKE_3RDPARTY_DIR}/svmlightlib/)
    set(SUBSYS_DEPS ${SUBSYS_DEPS} svmlight)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${COMMON_INCLUDE_DIR})
include_directories(${COMMON_IMPL_DIR})
include_directories(${PCL_INCLUDE_DIRS}s)

OD_ADD_LIBRARY(${SUBSYS_NAME} SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})
install(DIRECTORY ${COMMON_INCLUDE_DIR}/od DESTINATION ${OD_INSTALL_INCLUDE_DIR} COMPONENT ${LIB_NAME})
install(DIRECTORY ${COMMON_IMPL_DIR}/od DESTINATION ${OD_INSTALL_INCLUDE_DIR} COMPONENT ${LIB_NAME})

