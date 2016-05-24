set(SUBSYS_NAME pointcloud_global_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The global detector for point cloouds")
set(SUBSYS_DEPS od_common ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} siftgpu)

set(build TRUE)
#PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
#PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

if(build)

    set(INCLUDES
        "${DETECTORS_INCLUDE_DIR}/detectors/global3D/ODPointCloudGlobalMatching.h"
        "${DETECTORS_INCLUDE_DIR}/detectors/global3D/detection/ODCADDetector3DGlobal.h"
        "${DETECTORS_INCLUDE_DIR}/detectors/global3D/training/ODCADDetectTrainer3DGlobal.h"
        )

    set(IMPL_INCLUDES
        "${DETECTORS_IMPL_DIR}/detection/ODCADDetector3DGlobal.hpp"
        )

    set(SOURCES
        "ODPointCloudGlobalMatching.cpp"
        "detection/ODCADDetector3DGlobal.cpp"
        "training/ODCADDetectTrainer3DGlobal.cpp"
        )

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${DETECTORS_IMPL_DIR})
    include_directories(${COMMON_INCLUDE_DIR})

    OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${SOURCES} INCS ${INCLUDES} ${IMPL_INCLUDES})
    install(FILES ${INCLUDES} DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${SUBSYS_NAME} COMPONENT ${LIB_NAME})

    if(SUBSYS_DEPS)
      target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS})
    endif(SUBSYS_DEPS)

    #PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")


endif(build)