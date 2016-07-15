set(SUBSYS_NAME global_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "global feature based detection in 2D images")

set(SUBSYS_DEPS od_common svmlight)

if(BUILD_GLOBAL_2D_DETECTION)

    set(SOURCES
            "ODFaceRecognizer.cpp"
            "detection/ODCascadeDetector.cpp"
            "detection/ODHOGDetector.cpp"
            "training/ODHOGTrainer.cpp"
    )
    
    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${CMAKE_3RDPARTY_DIR}/svmlightlib/)
    include_directories(${PCL_INCLUDE_DIRS})

    OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

endif(BUILD_GLOBAL_2D_DETECTION)