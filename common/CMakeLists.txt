set(SUBSYS_NAME common)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "The core module of OpenDetection having the pipeline logic")
set(SUBSYS_DEPS ${OpenCV_LIBS} ${PCL_LIBRARIES})
set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)

if(WITH_GPU)
  set(SUBSYS_DEPS ${SUBSYS_DEPS} siftgpu)
endif()

set(build TRUE)

if(build)

    set(INCLUDES
            "include/common/pipeline/ObjectDetector.h"
            "include/common/pipeline/ODAlgorithmBase.h"
            "include/common/pipeline/ODScene.h"
            "include/common/pipeline/ODDetection.h"
            "include/common/pipeline/ODTrainer.h"
            "include/common/pipeline/ODDetector.h"
            "include/common/utils/ODFrameGenerator.h"
            "include/common/utils/utils.h"
            "include/common/utils/ODFeatureDetector2D.h"
            "include/common/bindings/svmlight.h"
        )

    set(IMPL_INCLUDES "")

    set(SOURCES
            "src/pipeline/ObjectDetector.cpp"
            "src/pipeline/ODScene.cpp"
            "src/utils/utils.cpp"
            "src/utils/ODFeatureDetector2D.cpp"
        )

    include_directories(${COMMON_INCLUDE_DIR})

    OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${SOURCES} INCS ${INCLUDES} ${IMPL_INCLUDES})
    install(FILES ${INCLUDES} DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${SUBSYS_NAME} COMPONENT ${LIB_NAME})

    if(SUBSYS_DEPS)
      target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS})
    endif(SUBSYS_DEPS)

endif(build)