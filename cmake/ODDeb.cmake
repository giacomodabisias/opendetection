set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Giacomo Dabisias") 
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Opendetection library installation package")
set(CPACK_DEBIAN_PACKAGE_VERSION {OD_MAJOR_VERSION}.${OD_MINOR_VERSION}.${OD_BUILD_VERSION})

# Set the correct architecture for the deb
execute_process(COMMAND dpkg --print-architecture COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
if(ARCHITECTURE)
	set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${ARCHITECTURE}) 
endif()

include(CPack)