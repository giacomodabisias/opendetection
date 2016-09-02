set(SUBSYS_NAME global_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "global feature based detection in 2D images")

set(SUBSYS_DEPS od_common)

if(BUILD_GLOBAL_2D_DETECTION)

    if(WITH_GPU)
        set(SUBSYS_DEPS ${SUBSYS_DEPS} od_gpu_global_image_detector)
    endif()

    set(SOURCES
        "FaceRecognizer.cpp"
        "detection/CascadeDetector.cpp"
        "detection/CascadeDetectorImpl.cpp"           
    )

if(0)
    if(WITH_CAFFE AND Caffe_FOUND)
        set(SOURCES ${SOURCES} 
            "detection/ConvClassification.cpp"
            "localization/SelectiveSearchBase.cpp"
            "localization/SelectiveSearchModel.cpp"
            "training/HOGTrainer.cpp"
        )
        set(SUBSYS_DEPS ${SUBSYS_DEPS} ${Caffe_LIBRARIES})
        include_directories(${CAFFE_INCLUDE_DIR})
        if(WITH_GPU)
            include_directories(${CUDA_INCLUDE_DIRS})
        endif()

        if(WITH_GTKMM AND GTKMM_FOUND)
            set(SOURCES ${SOURCES}  "training/Solver.cpp" 
                                    "training/ConvTrainer.cpp"
                                    "training/Network.cpp"  
                                    )
            include_directories(${GTKMM_INCLUDE_DIRS})
            link_directories(${GTKMM_LIBRARY_DIRS})
            set(SUBSYS_DEPS ${SUBSYS_DEPS} ${GTKMM_LIBRARIES})
        endif()

    endif()
endif(0)

    if(WITH_GTKMM AND GTKMM_FOUND)
         set(SOURCES ${SOURCES}    "annotation/Annotation.cpp"
                                   "annotation/Annotator.cpp")
    endif()

    if(WITH_SVMLIGHT)
        set(SOURCES ${SOURCES} "training/HOGTrainer.cpp" "detection/HOGDetector.cpp")
        include_directories(${CMAKE_3RDPARTY_DIR}/svmlightlib/)
        set(SUBSYS_DEPS ${SUBSYS_DEPS} svmlight)
    endif()
    
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})   
    include_directories(${PCL_INCLUDE_DIRS})

    OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

endif(BUILD_GLOBAL_2D_DETECTION)