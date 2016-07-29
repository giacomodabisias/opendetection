set(SUBSYS_NAME misc_detector)
set(LIB_NAME od_${SUBSYS_NAME})
set(SUBSYS_DESC "Multi algorithm detector")
set(SUBSYS_DEPS od_common od_global_image_detector od_pointcloud_global_detector)

if(BUILD_MISC_DETECTION)
    if(TARGET od_global_image_detector AND TARGET od_pointcloud_global_detector)
	    set(SOURCES
	        "detection/DetectorMultiAlgo.cpp"
	        )

	    include_directories(${DETECTORS_IMPL_DIR})
	    include_directories(${DETECTORS_INCLUDE_DIR})
	    include_directories(${COMMON_INCLUDE_DIR})
	    include_directories(${COMMON_IMPL_DIR})
	    include_directories(${OpenCV_INCLUDE_DIRS})
	    include_directories(${CMAKE_3RDPARTY_DIR}/svmlightlib/)
	    include_directories(${PCL_INCLUDE_DIRS})

	    OD_ADD_LIBRARY("${SUBSYS_NAME}" SRCS ${SOURCES} LINK_WITH ${SUBSYS_DEPS})

	else()
		message("!!! BUILD_MISC_DETECTION is set to on but BUILD_GLOBAL_2D_DETECTION and BUILD_GLOBAL_3D_DETECTION is necessary to build BUILD_MISC_DETECTION")
	endif(TARGET od_global_image_detector AND TARGET od_pointcloud_global_detector)

endif(BUILD_MISC_DETECTION)