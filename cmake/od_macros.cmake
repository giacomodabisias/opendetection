# global installation type
set(OD_LIB_TYPE SHARED)

macro(OD_ADD_LIBRARY1 _name _srcs _incs _impl_incs)

    set(lib_name "od_${_name}")

    message(input to od_add_library: ${OD_LIB_TYPE} ${_srcs} ${_incs} ${_impl_incs})
    add_library(${lib_name} ${OD_LIB_TYPE} ${_srcs} ${_incs} ${_impl_incs})

    # allways link libs:
    target_link_libraries(${lib_name} ${Boost_LIBRARIES})

    # target properties
    set_target_properties(${lib_name} PROPERTIES
        VERSION ${OD_VERSION}
        SOVERSION ${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}
        )

    # Install library
    install(TARGETS ${lib_name}
        RUNTIME DESTINATION ${OD_INSTALL_RUNTIME_DIR} COMPONENT ${lib_name}
        LIBRARY DESTINATION ${OD_INSTALL_LIBRARY_DIR} COMPONENT ${lib_name}
        ARCHIVE DESTINATION ${OD_INSTALL_ARCHIVE_DIR} COMPONENT ${lib_name})

    #install includes
    install(FILES ${_incs} ${_impl_incs}
            DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${_name}
            COMPONENT ${lib_name})

endmacro(OD_ADD_LIBRARY1)

macro(OD_ADD_LIBRARY_ALL _name )

    set(lib_name "od_${_name}")

    set(options)
    set(oneValueArgs)
    set(multiValueArgs SRCS)
    cmake_parse_arguments(OD_ADD_LIBRARY_ALL "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_library(${lib_name} ${OD_LIB_TYPE} ${OD_ADD_LIBRARY_ALL_SRCS})

    # allways link libs:
    target_link_libraries(${lib_name} ${Boost_LIBRARIES})

    # target properties
    set_target_properties(${lib_name} PROPERTIES
        VERSION ${OD_VERSION}
        SOVERSION ${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}
        )

    # Install library
    install(TARGETS ${lib_name}
        RUNTIME DESTINATION ${OD_INSTALL_RUNTIME_DIR} COMPONENT ${lib_name}
        LIBRARY DESTINATION ${OD_INSTALL_LIBRARY_DIR} COMPONENT ${lib_name}
        ARCHIVE DESTINATION ${OD_INSTALL_ARCHIVE_DIR} COMPONENT ${lib_name})


endmacro(OD_ADD_LIBRARY_ALL)

macro(OD_ADD_LIBRARY _name)

    set(lib_name "od_${_name}")

    include_directories("${OD_SOURCE_DIR}")

    #message(input to od_add_library: ${OD_LIB_TYPE} ${ARGN})
    add_library(${lib_name} ${OD_LIB_TYPE} ${ARGN})


    # allways link libs:
    target_link_libraries(${lib_name} ${Boost_LIBRARIES})

    # target properties
    set_target_properties(${lib_name} PROPERTIES
        VERSION ${OD_VERSION}
        SOVERSION ${OD_MAJOR_VERSION}.${OD_MINOR_VERSION}
        )

    # Install library
    install(TARGETS ${lib_name}
        RUNTIME DESTINATION ${OD_INSTALL_RUNTIME_DIR} COMPONENT ${lib_name}
        LIBRARY DESTINATION ${OD_INSTALL_LIBRARY_DIR} COMPONENT ${lib_name}
        ARCHIVE DESTINATION ${OD_INSTALL_ARCHIVE_DIR} COMPONENT ${lib_name})

endmacro(OD_ADD_LIBRARY)


macro(OD_ADD_EXAMPLE _name)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs FILES LINK_WITH INCLUDE)
    cmake_parse_arguments(OD_ADD_EXAMPLE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
    include_directories("${OD_SOURCE_DIR}" ${OD_ADD_EXAMPLE_INCLUDE})
    add_executable(${_name} ${OD_ADD_EXAMPLE_FILES})
    target_link_libraries(${_name} ${OD_ADD_EXAMPLE_LINK_WITH})
endmacro(OD_ADD_EXAMPLE)


macro(OD_ADD_INCLUDES _name)
    #install includes
    install(FILES ${ARGN}
            DESTINATION ${OD_INSTALL_INCLUDE_DIR}/${_name}
            COMPONENT ${lib_name})

endmacro(OD_ADD_INCLUDES)

MACRO(HEADER_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list *.h)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

MACRO(CMAKE_SUBDIR_LIST result)
  file (GLOB result */CMakeLists.txt)
ENDMACRO()

MACRO(SUBDIR_LIST retval curdir return_relative)
  file(GLOB sub-dir RELATIVE ${curdir} *)
  set(list_of_dirs "")
  foreach(dir ${sub-dir})
    if(IS_DIRECTORY ${curdir}/${dir})
      if (${return_relative})
        set(list_of_dirs ${list_of_dirs} ${dir})
      else()
        set(list_of_dirs ${list_of_dirs} ${curdir}/${dir})
      endif()
    endif()
  endforeach()
  set(${retval} ${list_of_dirs})
endmacro()
