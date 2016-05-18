set(SUBSYS_NAME global_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "global feature based detection in 2D images")

set(SUBSYS_DEPS od_common ${OpenCV_LIBS})

set(build TRUE)
#PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
#PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})


if(build)
    set(IMPL_INCLUDES "")

    if(WITH_SVMLIGHT)
        set(SOURCES
                "ODFaceRecognizer.cpp"
                "detection/ODHOGDetector.cpp"
                "detection/ODCascadeDetector.cpp"
                "training/ODHOGTrainer.cpp"
                )
        set(INCLUDES
            "${DETECTORS_INCLUDE_DIR}/detectors/global2D/DFaceRecognizer.h"
            "${DETECTORS_INCLUDE_DIR}/detectors/global2D/detection/ODHOGDetector.h"
            "${DETECTORS_INCLUDE_DIR}/detectors/global2D/detection/ODCascadeDetector.h"
            "${DETECTORS_INCLUDE_DIR}/detectors/global2D/training/ODHOGTrainer.h"
            )
    else()
        set(SOURCES
                "ODFaceRecognizer.cpp"
                "detection/ODCascadeDetector.cpp"
        )

        set(INCLUDES
            "${DETECTORS_INCLUDE_DIR}/detectors/global2D/ODFaceRecognizer.h"
            "${DETECTORS_INCLUDE_DIR}/detectors/global2D/detection/ODCascadeDetector.h"
            )

    endif()
        
    include_directories(${DETECTORS_INCLUDE_DIR}/detectors/global2D)
    include_directories(${OD_SOURCE_DIR}/common/include)

    OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${SOURCES} INCS ${INCLUDES} ${impl_incs})
    install(FILES ${INCLUDES} DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${SUBSYS_NAME} COMPONENT ${LIB_NAME} )

    if(WITH_SVMLIGHT)
        if(SUBSYS_DEPS)
          target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS} svm)
        endif(SUBSYS_DEPS)
    else()
        if(SUBSYS_DEPS)
          target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS} )
        endif(SUBSYS_DEPS)
    endif()

    

    #PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")


endif(build)