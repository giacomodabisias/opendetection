if(WITH_GPU)
	set(GPU_DETECTORS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
	set(GPU_DETECTORS_INCLUDE_DIR ${GPU_DETECTORS_DIR}/include)

	include_directories(${CMAKE_3RDPARTY_DIR}/SiftGPU/src/SiftGPU/)

	add_subdirectory("src/global2D")

	install(DIRECTORY ${GPU_DETECTORS_INCLUDE_DIR}/od DESTINATION ${OD_INSTALL_INCLUDE_DIR})
endif()
