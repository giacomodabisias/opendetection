#add mandatory 3rdparty libraries
add_subdirectory(pugixml_build)

add_subdirectory(svmlightlib)

if(WITH_GPU)
  add_subdirectory(SiftGPU)
endif(WITH_GPU)