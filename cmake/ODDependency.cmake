find_package(PCL REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(VTK REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)

# Needed by od_common. There is a circular dependency between od_common and od_gpu_common include dirs.
include_directories(GPU_COMMON_INCLUDE_DIRS ${OD_SOURCE_DIR}/gpu/common/include)
include_directories(GPU_DETECTORS_INCLUDE_DIRS ${OD_SOURCE_DIR}/gpu/detectors/include)