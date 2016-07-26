if(WITH_GPU)
	set(GPU_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR})
	set(GPU_COMMON_INCLUDE_DIR ${GPU_COMMON_DIR}/include)

	set(SUBSYS_NAME gpu_common)
	set(LIB_NAME od_${SUBSYS_NAME})
	set(SUBSYS_DESC "common gpu feature utilities")

	set(SUBSYS_DEPS siftgpu)

	set(SOURCES
	    "src/utils/ODFeatureDetector2D.cpp"
	)

	include_directories(${GPU_COMMON_INCLUDE_DIR})
	include_directories(${COMMON_INCLUDE_DIR})
	include_directories(${OpenCV_INCLUDE_DIRS})
	include_directories(${CMAKE_3RDPARTY_DIR}/SiftGPU/src/SiftGPU/)

	OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

	install(DIRECTORY ${GPU_COMMON_INCLUDE_DIR}/od DESTINATION ${OD_INSTALL_INCLUDE_DIR})
endif()
