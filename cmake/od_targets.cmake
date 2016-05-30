# installation structure
if(NOT OD_INSTALL_RUNTIME_DIR)
  set(OD_INSTALL_RUNTIME_DIR bin)
endif()

if(NOT OD_INSTALL_LIBRARY_DIR)
  set(OD_INSTALL_LIBRARY_DIR lib)
endif()

if(NOT OD_INSTALL_ARCHIVE_DIR)
  set(OD_INSTALL_ARCHIVE_DIR lib)
endif()

if(NOT OD_INSTALL_INCLUDE_DIR)
  set(OD_INSTALL_INCLUDE_DIR include/od-${OD_VERSION})
endif()

if(NOT OD_INSTALL_DATA_DIR)
  set(OD_INSTALL_DATA_DIR share/od-${OD_VERSION})
endif()

if(NOT OD_INSTALL_DOC_DIR)
  set(OD_INSTALL_DOC_DIR share/doc/od-${OD_VERSION})
endif()

if(NOT OD_INSTALL_PACKAGE_DIR)
  set(OD_INSTALL_PACKAGE_DIR "lib/cmake/od-${OD_VERSION}")
endif()

if(NOT OD_INSTALL_DOXYGEN_DIR)
  set(OD_INSTALL_DOXYGEN_DIR ${OD_INSTALL_DOC_DIR}/doxygen)
endif()


