include_directories(${CMAKE_3RDPARTY_DIR}/SiftGPU/src/SiftGPU/)


option(camera_recognition_example "Build the camera recognition example" ON)
if(od_image_camera_example)
  if(TARGET od_local_image_detector)
  
    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})

    OD_ADD_EXAMPLE(od_image_camera 
                   FILES od_image_cadrecog_camera.cpp
                   LINK_WITH od_local_image_detector)
  else()
    message("!!! od_image_camera_example is set to on but BUILD_LOCAL_2D_DETECTION is necessary to build od_image_camera_example")
  endif(TARGET od_local_image_detector)
endif(od_image_camera_example)

##################################################################################
option(image_recognition_example "Build the image recognition example" ON)
if(image_recognition_example)
  if(TARGET od_local_image_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})
    include_directories(${CMAKE_3RDPARTY_DIR}/pugixml/src/)

    OD_ADD_EXAMPLE(od_example_files 
                   FILES od_image_cadrecog_files.cpp
                   LINK_WITH od_local_image_detector)
  else()
    message("!!! image_recognition_example is set to on but BUILD_LOCAL_2D_DETECTION is necessary to build image_recognition_example")
  endif(TARGET od_local_image_detector)
endif(image_recognition_example)

##################################################################################
option(hog_train_example "Build the hog train example" ON)
if(hog_train_example)
  if(TARGET od_global_image_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})

    OD_ADD_EXAMPLE(od_hog_train 
                   FILES od_hog_train.cpp
                   LINK_WITH od_global_image_detector)
  else()
    message("!!! hog_train_example is set to on but BUILD_GLOBAL_2D_DETECTION is necessary to build hog_train_example")
  endif(TARGET od_global_image_detector)
endif(hog_train_example)

##################################################################################
option(image_hog_files_example "Build the hog image example" ON)
if(image_hog_files_example)
  if(TARGET od_global_image_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})
    include_directories(${CMAKE_3RDPARTY_DIR}/svmlightlib/)

    OD_ADD_EXAMPLE(od_image_hog_files 
                   FILES od_image_hog_files.cpp
                   LINK_WITH od_global_image_detector)
  else()
    message("!!! image_hog_files_example is set to on but BUILD_GLOBAL_2D_DETECTION is necessary to build image_hog_files_example")
  endif(TARGET od_global_image_detector)
endif(image_hog_files_example)

##################################################################################
option(face_detection_files_example "Build the face detection example with file input" ON)
if(face_detection_files_example)
  if(TARGET od_global_image_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})

    set(SYS_DEP od_global_image_detector)

    if(WITH_GPU)
      include_directories(${GPU_DETECTORS_INCLUDE_DIR})
      set(SYS_DEP ${SYS_DEP} od_gpu_global_image_detector)
    endif()

    OD_ADD_EXAMPLE(od_face_detection_files 
                   FILES od_face_detector_files.cpp
                   LINK_WITH ${SYS_DEP})
  else()
    message("!!! face_detection_files_example is set to on but BUILD_GLOBAL_2D_DETECTION is necessary to build face_detection_files_example")
  endif(TARGET od_global_image_detector)
endif(face_detection_files_example)

##################################################################################
option(face_detection_cam_example "Build the face detection example with camera input" ON)
if(face_detection_cam_example)
  if(TARGET od_global_image_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})

    set(SYS_DEP od_global_image_detector)

    if(WITH_GPU)
      include_directories(${GPU_DETECTORS_INCLUDE_DIR})
      set(SYS_DEP ${SYS_DEP} od_gpu_global_image_detector)
    endif()

    OD_ADD_EXAMPLE(od_face_detection_cam 
                   FILES od_face_detector_cam.cpp
                   LINK_WITH ${SYS_DEP})
  else()
    message("!!! face_detection_cam_example is set to on but BUILD_GLOBAL_2D_DETECTION is necessary to build face_detection_cam_example")
  endif(TARGET od_global_image_detector)
endif(face_detection_cam_example)

##################################################################################
option(image_customhog_example "Build the custom hog example" ON)
if(image_customhog_example)
  if(TARGET od_global_image_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})

    OD_ADD_EXAMPLE(od_image_customhog 
                   FILES od_image_customhog.cpp
                   LINK_WITH od_global_image_detector)
    else()
      message("!!! image_customhog_example is set to on but BUILD_GLOBAL_2D_DETECTION is necessary to build image_customhog_example")
    endif(TARGET od_global_image_detector)
endif(image_customhog_example)

##################################################################################
option(multialgo_files_example "Build the multialgo_files example" ON)
if(multialgo_files_example)
  if(TARGET od_misc_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${DETECTORS_IMPL_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})
    include_directories(${OpenCV_INCLUDE_DIRS})

    OD_ADD_EXAMPLE(od_multialgo_files 
                   FILES od_multialgo_files.cpp
                   LINK_WITH od_misc_detector)
  else()
    message("!!! multialgo_files_example is set to on but BUILD_MISC_DETECTION is necessary to build multialgo_files_example")
  endif(TARGET od_misc_detector)
endif(multialgo_files_example)

##################################################################################
option(multialgo_pc_example "Build the multialgo_pc example" ON)
if(multialgo_pc_example)
  if(TARGET od_misc_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})

    OD_ADD_EXAMPLE(od_multialgo_pc 
                   FILES od_multialgo_pc.cpp
                   LINK_WITH od_misc_detector)
  else()
    message("!!! multialgo_pc_example is set to on but BUILD_MISC_DETECTION is necessary to build multialgo_pc_example")
  endif(TARGET od_misc_detector)
endif(multialgo_pc_example)

##################################################################################
option(cascade_cam_example "Build the cascade camera example" ON)
if(cascade_cam_example)
  if(TARGET od_global_image_detector)
  
    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})
    include_directories(${PCL_INCLUDE_DIRS})

    OD_ADD_EXAMPLE(od_cascade_cam 
                   FILES od_cascade_cam.cpp
                   LINK_WITH od_global_image_detector)
  else()
    message("!!! cascade_cam_example is set to on but BUILD_GLOBAL_2D_DETECTION is necessary to build cascade_cam_example")
  endif(TARGET od_global_image_detector)
endif(cascade_cam_example)

##################################################################################
option(cascade_files_example "Build the cascade file example" ON)
if(cascade_files_example)
  if(TARGET od_global_image_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})

    OD_ADD_EXAMPLE(od_cascade_files 
                   FILES od_cascade_files.cpp
                   LINK_WITH od_global_image_detector)
  else()
    message("!!! cascade_files_example is set to on but BUILD_GLOBAL_2D_DETECTION is necessary to build cascade_files_example")
  endif(TARGET od_global_image_detector)
endif(cascade_files_example)

##################################################################################
option(image_facerecog_example "Build the face recognition example" ON)
if(image_facerecog_example)
  if(TARGET od_global_image_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${COMMON_INCLUDE_DIR})
    include_directories(${COMMON_IMPL_DIR})

    OD_ADD_EXAMPLE(od_image_facerecog 
                   FILES od_image_facerecog.cpp
                   LINK_WITH od_global_image_detector)
  else()
    message("!!! image_facerecog_example is set to on but BUILD_GLOBAL_2D_DETECTION is necessary to build image_facerecog_example")
  endif(TARGET od_global_image_detector)
endif(image_facerecog_example)

##################################################################################
option(pc_global_example "Build the global detector example" ON)
if(pc_global_example)
  if(TARGET od_pointcloud_global_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${DETECTORS_IMPL_DIR})
    include_directories(${COMMON_INCLUDE_DIR})

    OD_ADD_EXAMPLE(od_example_pc_global 
                   FILES od_pc_global.cpp
                   LINK_WITH od_pointcloud_global_detector)
  else()
    message("!!! pc_global_example is set to on but BUILD_GLOBAL_3D_DETECTION is necessary to build pc_global_example")
  endif(TARGET od_pointcloud_global_detector)
endif(pc_global_example) 

##################################################################################
option(pc_global_files_example "Build the global detector file example" ON)
if(pc_global_files_example)
  if(TARGET od_pointcloud_global_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${DETECTORS_IMPL_DIR})
    include_directories(${COMMON_INCLUDE_DIR})

    OD_ADD_EXAMPLE(od_example_pc_global_files 
                   FILES od_pc_global_files.cpp
                   LINK_WITH od_pointcloud_global_detector)
  else()
    message("!!! pc_global_files_example is set to on but BUILD_GLOBAL_3D_DETECTION is necessary to build pc_global_files_example")
  endif(TARGET od_pointcloud_global_detector)
endif(pc_global_files_example)

##################################################################################
option(pc_global_real_time_example "Build the global detector real time example" ON)
if(pc_global_real_time_example)
  if(TARGET od_pointcloud_global_detector)

    include_directories(${DETECTORS_INCLUDE_DIR})
    include_directories(${DETECTORS_IMPL_DIR})
    include_directories(${COMMON_INCLUDE_DIR})

    OD_ADD_EXAMPLE(od_example_pc_global_real_time 
                   FILES od_pc_global_real_time.cpp
                   LINK_WITH od_pointcloud_global_detector)
  else()
    message("!!! pc_global_real_time_example is set to on but BUILD_GLOBAL_3D_DETECTION is necessary to build pc_global_real_time_example")
  endif(TARGET od_pointcloud_global_detector)
endif(pc_global_real_time_example)

##################################################################################
option(framegenerator_example "Build the frame generator example" ON)
if(framegenerator_example)

  include_directories(${COMMON_INCLUDE_DIR})
  include_directories(${COMMON_IMPL_DIR})
  include_directories(${PCL_INCLUDE_DIRS})

  OD_ADD_EXAMPLE(od_framegenerator 
                 FILES od_framegenerator.cpp
                 LINK_WITH od_common)
endif()



