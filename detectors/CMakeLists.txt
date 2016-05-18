set(DETECTORS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DETECTORS_INCLUDE_DIR ${DETECTORS_DIR}/include)
set(DETECTORS_IMPL_DIR ${DETECTORS_DIR}/impl)

add_subdirectory(simple_ransac_detection)
add_subdirectory("src/local2D")
add_subdirectory("src/global2D")
add_subdirectory("src/global3D")
add_subdirectory("src/misc")
