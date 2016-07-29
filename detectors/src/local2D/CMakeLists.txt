set(SUBSYS_NAME local_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The local feature matching based detector")
set(SUBSYS_DEPS od_common pugixml)

if(BUILD_LOCAL_2D_DETECTION)
    
    set(SOURCES
        "ImageLocalMatching.cpp"
        "training/CADRecogTrainerSnapshotBased.cpp"
        "detection/CADRecognizer2DLocal.cpp"
        "simple_ransac_detection/CsvReader.cpp"
        "simple_ransac_detection/CsvWriter.cpp"
        "simple_ransac_detection/ModelRegistration.cpp"
        "simple_ransac_detection/Mesh.cpp"
        "simple_ransac_detection/Model.cpp"
        "simple_ransac_detection/PnPProblem.cpp"
        "simple_ransac_detection/Utils.cpp"
        "simple_ransac_detection/RobustMatcher.cpp"
        )

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${CMAKE_3RDPARTY_DIR}/pugixml/src/)
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})
    include_directories(${PCL_INCLUDE_DIRS})

    OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

endif(BUILD_LOCAL_2D_DETECTION)