option(cad_recog_test_single_model "Build the cad recognition test" ON)
if(cad_recog_test_single_model)
	if(TARGET od_local_image_detector)

		include_directories(${OpenCV_INCLUDE_DIRS})
		include_directories(${DETECTORS_INCLUDE_DIR})
		include_directories(${COMMON_INCLUDE_DIR})
		include_directories(${COMMON_IMPL_DIR})
		include_directories(${CMAKE_3RDPARTY_DIR}/pugixml/src/)
		include_directories(${CMAKE_3RDPARTY_DIR}/SiftGPU/src/SiftGPU/)
		include_directories(${PCL_INCLUDE_DIRS})

		link_directories(${PCL_LIBRARY_DIRS})
		link_directories(${OPENCV_LIBRARY_DIRS})

		OD_ADD_EXAMPLE(odcadrecog_test_single_model 
		              FILES cadrecog2D/test_single_db_single_model.cpp
		              LINK_WITH od_local_image_detector
		)
	else()
	  message("!!! cad_recog_test_single_model is set to on but BUILD_GLOBAL_2D_DETECTION is necessary to build cad_recog_test_single_model")
	endif(TARGET od_local_image_detector)
endif(cad_recog_test_single_model)

##################################################################################
option(multihog_app "Build the multihog app" ON)
if(od_multihog_app)
	if(TARGET od_global_image_detector)	

		include_directories(${OpenCV_INCLUDE_DIRS})
		include_directories(${DETECTORS_INCLUDE_DIR})
		include_directories(${COMMON_INCLUDE_DIR})

		link_directories(${PCL_LIBRARY_DIRS})
  		link_directories(${OPENCV_LIBRARY_DIRS})

		OD_ADD_EXAMPLE(od_multihog_app 
		               FILES global2D/multihog_app.cpp
		               LINK_WITH od_global_image_detector
		)
	else()
	  message("!!! multihog_app is set to on but BUILD_GLOBAL_2D_DETECTION is necessary to build od_multihog_app")
	endif(TARGET od_global_image_detector)
endif(od_multihog_app)

##################################################################################
set(SOURCE_FILES version/opendetection.cpp)
include_directories(${CMAKE_OD_GENERATED_DIR})
add_executable(opendetection ${SOURCE_FILES})
