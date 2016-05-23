set(DETECTORS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(SIMPLE_RANSAC_INCLUDE_DIR ${DETECTORS_DIR})
set(DETECTORS_INCLUDE_DIR ${DETECTORS_DIR}/include)
set(DETECTORS_IMPL_DIR ${DETECTORS_DIR}/impl)

#Add in parent scope to build examples
set(DETECTORS_INCLUDE_DIR ${DETECTORS_DIR}/include PARENT_SCOPE)
set(DETECTORS_IMPL_DIR ${DETECTORS_DIR}/impl PARENT_SCOPE)
set(SIMPLE_RANSAC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

add_subdirectory(simple_ransac_detection)
add_subdirectory("src/local2D")
add_subdirectory("src/global2D")
add_subdirectory("src/global3D")
add_subdirectory("src/misc")
