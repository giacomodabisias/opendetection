set(SUBSYS_NAME pointcloud_global_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The global detector for point clouds")
set(SUBSYS_DEPS od_common)

if(BUILD_GLOBAL_3D_DETECTION)

    if(WITH_GPU)
        set(SUBSYS_DEPS ${SUBSYS_DEPS} siftgpu)
    endif()

    set(SOURCES
        "PointCloudGlobalMatching.cpp"
        "training/CADDetectTrainer3DGlobal.cpp"
        "detection/CADDetector3DGlobal.cpp"
    )

    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${DETECTORS_IMPL_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})
    include_directories(${PCL_INCLUDE_DIRS})

    OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

endif(BUILD_GLOBAL_3D_DETECTION)