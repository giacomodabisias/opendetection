set(SUBSYS_NAME gpu_global_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "global gpu feature based detection in 2D images")

set(SUBSYS_DEPS od_common)


set(SOURCES
    "detection/ODCascadeDetector.cpp"
)

include_directories(${GPU_DETECTORS_INCLUDE_DIR})
include_directories(${COMMON_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

