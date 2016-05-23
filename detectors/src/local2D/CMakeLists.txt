set(SUBSYS_NAME local_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The local feature matching based detector")

set(SUBSYS_DEPS od_common ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS})

include_directories(${CMAKE_SOURCE_DIR}/3rdparty/pugixml/src/)
link_directories(${CMAKE_SOURCE_DIR}/3rdparty/pugixml/build/)

set(build TRUE)
#PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
#PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})


if(build)
    
    set(INCLUDES
        "${DETECTORS_INCLUDE_DIR}/detectors/local2D/ODImageLocalMatching.h"
        "${DETECTORS_INCLUDE_DIR}/detectors/local2D/training/ODCADRecogTrainerSnapshotBased.h"
        "${DETECTORS_INCLUDE_DIR}/detectors/local2D/detection/ODCADRecognizer2DLocal.h"
        )

    set(IMPL_INCLUDES "")

    set(SOURCES
        "ODImageLocalMatching.cpp"
        "training/ODCADRecogTrainerSnapshotBased.cpp"
        "detection/ODCADRecognizer2DLocal.cpp"
        ${SIMPLE_RANSAC_SRC_FILES}
        )

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${SIMPLE_RANSAC_INCLUDE_DIR})

    OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${SOURCES} INCS ${INCLUDES} ${IMPL_INCLUDES})
    install(FILES ${INCLUDES} DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${SUBSYS_NAME} COMPONENT ${LIB_NAME})

    if(SUBSYS_DEPS)
      target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS} pugixml)
    endif(SUBSYS_DEPS)

    #PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

endif(build)