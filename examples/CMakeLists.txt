if(WITH_EXAMPLES)
	SUBDIR_LIST(EXAMPLE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} "")
	foreach(DIR ${EXAMPLE_DIRS})
		add_subdirectory(${DIR})
	endforeach()
endif()