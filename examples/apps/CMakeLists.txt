include_directories(${DETECTORS_INCLUDE_DIR})
include_directories(${COMMON_INCLUDE_DIR})
include_directories(${SIMPLE_RANSAC_INCLUDE_DIR})

OD_ADD_EXAMPLE(odcadrecog_test_single_model FILES cadrecog2D/od_test_single_db_single_model.cpp
                LINK_WITH od_common od_local_image_detector)

include_directories("${OD_BINARY_DIR}/generated")
set(SOURCE_FILES version/opendetection.cpp)
add_executable(opendetection ${SOURCE_FILES})

if(WITH_SVMLIGHT)
	OD_ADD_EXAMPLE(od_multihog_app FILES global2D/od_multihog_app.cpp
    LINK_WITH od_common od_global_image_detector)
endif()