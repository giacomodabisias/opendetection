set(SUBSYS_NAME global_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "global feature based detection in 2D images")

set(SUBSYS_DEPS od_common)

if(BUILD_GLOBAL_2D_DETECTION)

    if(WITH_GPU)
        set(SUBSYS_DEPS ${SUBSYS_DEPS} od_gpu_global_image_detector)
    endif()

    set(SOURCES
            "ODFaceRecognizer.cpp"
            "detection/ODCascadeDetector.cpp"
            "detection/ODCascadeDetectorImpl.cpp"
            
    )

    if(WITH_SVMLIGHT)
        set(SOURCES ${SOURCES} "training/ODHOGTrainer.cpp" "detection/ODHOGDetector.cpp")
        include_directories(${CMAKE_3RDPARTY_DIR}/svmlightlib/)
        set(SUBSYS_DEPS ${SUBSYS_DEPS} svmlight)
    endif()
    
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${PCL_INCLUDE_DIRS})

    OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

endif(BUILD_GLOBAL_2D_DETECTION)