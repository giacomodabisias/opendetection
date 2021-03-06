find_package(PCL REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(VTK REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)

if(0)
if(WITH_CAFFE)
  find_package(Caffe REQUIRED)
  if(Caffe_FOUND)
	add_definitions(${Caffe_DEFINITIONS})
	if(WITH_GPU)
		find_package(CUDA REQUIRED)
	endif()
	if(WITH_GTKMM)
		find_package(PkgConfig) 
		pkg_check_modules(GTKMM gtkmm-3.0)
	endif()
  else()
  	message("Caffe not found!")
  endif()
endif()
endif(0)

# Needed by od_common. There is a circular dependency between od_common and od_gpu_common include dirs.
include_directories(GPU_COMMON_INCLUDE_DIRS ${OD_SOURCE_DIR}/gpu/common/include)
include_directories(GPU_DETECTORS_INCLUDE_DIRS ${OD_SOURCE_DIR}/gpu/detectors/include)