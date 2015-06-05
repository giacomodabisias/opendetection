set(SUBSYS_NAME common)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The core module of OpenDetection having the pipeline logic")
set(SUBSYS_DEPS ${OpenCV_LIBS})

set(build TRUE)
#PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
#PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})


if(build)

    set(submodules
          pipeline
    )

    set(incs
        "pipeline/ObjectDetector.h"
        "pipeline/ImageScene.h"
        )

    set(impl_incs
        )

    set(srcs
        "pipeline/ObjectDetector.cpp"
        "pipeline/ImageScene.cpp"
        )




    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/pipeline")

    OD_ADD_LIBRARY("${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    OD_ADD_INCLUDES("${SUBSYS_NAME}" ${incs} ${impl_incs})

    if(SUBSYS_DEPS)
      target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS})
    endif(SUBSYS_DEPS)

    #PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")


endif(build)