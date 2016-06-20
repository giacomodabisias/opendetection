set(SUBSYS_NAME local_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The local feature matching based detector")
set(SUBSYS_DEPS od_common ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} pugixml)

set(build TRUE)

if(build)
    
    set(SOURCES
        "ODImageLocalMatching.cpp"
        "training/ODCADRecogTrainerSnapshotBased.cpp"
        "detection/ODCADRecognizer2DLocal.cpp"
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
    include_directories(${COMMON_INCLUDE_DIR})

    OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

endif(build)