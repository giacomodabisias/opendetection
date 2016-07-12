set(SUBSYS_NAME misc_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "Multi algorithm detector")
set(SUBSYS_DEPS od_common od_global_image_detector od_pointcloud_global_detector)

set(build TRUE)

if(build)
    
    set(SOURCES
        "detection/ODDetectorMultiAlgo.cpp"
        )

    include_directories(${DETECTORS_IMPL_DIR})
    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${CMAKE_3RDPARTY_DIR}/svmlightlib/)

    OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

endif(build)