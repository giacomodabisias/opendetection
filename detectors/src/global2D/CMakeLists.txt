set(SUBSYS_NAME global_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "global feature based detection in 2D images")

set(SUBSYS_DEPS od_common svmlight)

set(build TRUE)

if(build)

    set(SOURCES
            "ODFaceRecognizer.cpp"
            "detection/ODCascadeDetector.cpp"
            "detection/ODHOGDetector.cpp"
            "detection/ODFaceDetector.cpp"
            "training/ODHOGTrainer.cpp"
    )
    
    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${CMAKE_3RDPARTY_DIR}/svmlightlib/)

    OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

endif(build)