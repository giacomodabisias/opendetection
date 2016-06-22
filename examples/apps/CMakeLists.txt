include_directories(${DETECTORS_INCLUDE_DIR})
include_directories(${COMMON_INCLUDE_DIR})
include_directories(${COMMON_IMPL_DIR})

include_directories(${CMAKE_3RDPARTY_DIR}/pugixml/src/)
include_directories(${OD_BINARY_DIR}/generated)
include_directories(${CMAKE_3RDPARTY_DIR}/svmlightlib/)

OD_ADD_EXAMPLE(odcadrecog_test_single_model 
              FILES cadrecog2D/od_test_single_db_single_model.cpp
              LINK_WITH od_common od_local_image_detector
)


set(SOURCE_FILES version/opendetection.cpp)
add_executable(opendetection ${SOURCE_FILES})

OD_ADD_EXAMPLE(od_multihog_app 
               FILES global2D/od_multihog_app.cpp
               LINK_WITH od_common od_global_image_detector
)

OD_ADD_EXAMPLE(viewer_test 
               FILES visualization/ODVisualizationTest.cpp
               LINK_WITH od_common
)
