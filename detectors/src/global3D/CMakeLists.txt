set(SUBSYS_NAME pointcloud_global_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The global detector for point cloouds")
set(SUBSYS_DEPS od_common ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} siftgpu)

set(build TRUE)

if(build)

    set(SOURCES
        "ODPointCloudGlobalMatching.cpp"
        "detection/ODCADDetector3DGlobal.cpp"
        "training/ODCADDetectTrainer3DGlobal.cpp"
        )

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${DETECTORS_IMPL_DIR})
    include_directories(${COMMON_INCLUDE_DIR})

    OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${SOURCES})

    if(SUBSYS_DEPS)
      target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS})
    endif(SUBSYS_DEPS)

    #PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

endif(build)