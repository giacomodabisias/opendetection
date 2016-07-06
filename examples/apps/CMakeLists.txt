
option(cad_recog_test_single_model "Build the cad recognition test" ON)
if(cad_recog_test_single_model)

	include_directories(${DETECTORS_INCLUDE_DIR})
	include_directories(${COMMON_INCLUDE_DIR})
	include_directories(${CMAKE_3RDPARTY_DIR}/pugixml/src/)

	OD_ADD_EXAMPLE(odcadrecog_test_single_model 
	              FILES cadrecog2D/od_test_single_db_single_model.cpp
	              LINK_WITH od_local_image_detector
	)
endif()

##################################################################################
option(multihog_app "Build the multihog app" ON)
if(od_multihog_app)
	
	include_directories(${DETECTORS_INCLUDE_DIR})
	include_directories(${COMMON_INCLUDE_DIR})

	OD_ADD_EXAMPLE(od_multihog_app 
	               FILES global2D/od_multihog_app.cpp
	               LINK_WITH od_global_image_detector
	)
endif()

##################################################################################
option(viewer_test "Build the viewer test" ON)
if(viewer_test)

	include_directories(${COMMON_IMPL_DIR})

	OD_ADD_EXAMPLE(viewer_test 
	               FILES visualization/ODVisualizationTest.cpp
	               LINK_WITH od_common
	)
endif()

##################################################################################
set(SOURCE_FILES version/opendetection.cpp)
include_directories(${OD_BINARY_DIR}/generated)
add_executable(opendetection ${SOURCE_FILES})
