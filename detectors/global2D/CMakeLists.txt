set(SUBSYS_NAME global_image_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "global feature based detection in 2D images")

set(SUBSYS_DEPS od_common ${OpenCV_LIBS})

set(build TRUE)
#PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
#PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})


if(build)


    set(impl_incs
        )
if(WITH_SVMLIGHT)
    set(srcs
            ODFaceRecognizer.cpp
            "detection/ODHOGDetector.cpp"
            "detection/ODCascadeDetector.cpp"
            "training/ODHOGTrainer.cpp"
            )
    set(incs
        "ODFaceRecognizer.h"
        "detection/ODHOGDetector.h"
        "detection/ODCascadeDetector.h"
        "training/ODHOGTrainer.h"
        )


else()
    set(srcs
            ODFaceRecognizer.cpp
            "detection/ODCascadeDetector.cpp"
    )

    set(incs
        "ODFaceRecognizer.h"
        "detection/ODCascadeDetector.h"
        )

endif()
    


    OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${srcs} INCS ${incs} ${impl_incs})
    install(FILES ${incs} DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${SUBSYS_NAME} COMPONENT ${LIB_NAME} )

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