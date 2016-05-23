set(SIMPLE_RANSAC_SRC ${CMAKE_CURRENT_SOURCE_DIR})
set(SIMPLE_RANSAC_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

set(SIMPLE_RANSAC_SRC_FILES
        ${SIMPLE_RANSAC_SRC}/CsvReader.cpp
        ${SIMPLE_RANSAC_SRC}/CsvWriter.cpp
        ${SIMPLE_RANSAC_SRC}/ModelRegistration.cpp
        ${SIMPLE_RANSAC_SRC}/Mesh.cpp
        ${SIMPLE_RANSAC_SRC}/Model.cpp
        ${SIMPLE_RANSAC_SRC}/PnPProblem.cpp
        ${SIMPLE_RANSAC_SRC}/Utils.cpp
        ${SIMPLE_RANSAC_SRC}/RobustMatcher.cpp
PARENT_SCOPE)