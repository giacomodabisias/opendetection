set(SUBSYS_NAME local_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The local feature matching based detector")
set(SUBSYS_DEPS od_common pugixml)

set(build TRUE)

if(build)
    
    set(SOURCES
        "ODImageLocalMatching.cpp"
        "training/ODCADRecogTrainerSnapshotBased.cpp"
        "detection/ODCADRecognizer2DLocal.cpp"
        "simple_ransac_detection/ODCsvReader.cpp"
        "simple_ransac_detection/ODCsvWriter.cpp"
        "simple_ransac_detection/ODModelRegistration.cpp"
        "simple_ransac_detection/ODMesh.cpp"
        "simple_ransac_detection/ODModel.cpp"
        "simple_ransac_detection/ODPnPProblem.cpp"
        "simple_ransac_detection/ODUtils.cpp"
        "simple_ransac_detection/ODRobustMatcher.cpp"
        )

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${CMAKE_3RDPARTY_DIR}/pugixml/src/)
    include_directories(${COMMON_INCLUDE_DIR})

    OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

endif(build)