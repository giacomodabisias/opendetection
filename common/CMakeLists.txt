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

    set(IMPL_INCLUDES "")

    set(SOURCES
            "src/pipeline/ODObjectDetector.cpp"
            "src/pipeline/ODScene.cpp"
            "src/pipeline/ODDetection.cpp"
            "src/utils/ODUtils.cpp"
            "src/utils/ODFeatureDetector2D.cpp"
            "src/bindings/svmlight.cpp"
        )

    include_directories(${COMMON_INCLUDE_DIR})

    OD_ADD_LIBRARY_ALL("${SUBSYS_NAME}" SRCS ${SOURCES})
    install(DIRECTORY ${COMMON_INCLUDE_DIR}/od DESTINATION ${OD_INSTALL_INCLUDE_DIR} COMPONENT ${LIB_NAME})

    if(SUBSYS_DEPS)
      target_link_libraries("${LIB_NAME}" ${SUBSYS_DEPS})
    endif(SUBSYS_DEPS)

endif(build)