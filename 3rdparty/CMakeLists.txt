#add mandatory 3rdparty libraries
add_subdirectory(pugixml_build)

if(WITH_SVMLIGHT)
  add_subdirectory(svmlightlib)
endif(WITH_SVMLIGHT)

if(WITH_GPU)
  add_subdirectory(SiftGPU)
endif(WITH_GPU)