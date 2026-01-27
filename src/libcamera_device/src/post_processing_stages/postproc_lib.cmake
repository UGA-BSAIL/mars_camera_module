# Script to generate the postproc_lib.cpp file.
SET(POSTPROC_LIB_DIR ${PREFIX}/post_processing_stages)

configure_file(${CMAKE_CURRENT_LIST_DIR}/postproc_lib.cpp.in postproc_lib.cpp @ONLY)
message(STATUS "Postproc lib dir: " ${POSTPROC_LIB_DIR})
