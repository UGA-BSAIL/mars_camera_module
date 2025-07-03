# - Find TAPPAS library
# Find the native TAPPAS includes and library
# This module defines
#  TAPPAS_INCLUDE_DIR, where to find .h, etc.
#  TAPPAS_LIBRARIES, libraries to link against to use TAPPAS.
#  TAPPAS_FOUND, If false, do not try to use TAPPAS. also defined, but not for general use are
#  TAPPAS_POSTPROC_LIB_DIR, directory containing TAPPAS postprocessing libraries.
#

SET(TAPPAS_FOUND 0)

FIND_PATH(TAPPAS_INCLUDE_DIR
        NAMES hailo_common.hpp
        PATH_SUFFIXES hailo/tappas
        DOC "The TAPPAS include directory"
)

FIND_LIBRARY(DEBUG_LIBRARY
        NAMES debug
        PATH_SUFFIXES hailo/tappas/post_processes
        DOC "The debug library"
)

if (DEBUG_LIBRARY)
    get_filename_component(TAPPAS_POSTPROC_LIB_DIR ${DEBUG_LIBRARY} DIRECTORY)
    message(STATUS "Postproc library directory: ${TAPPAS_POSTPROC_LIB_DIR}")
else()
    message(FATAL_ERROR "Postproc library directory not found")
endif()

# handle the QUIETLY and REQUIRED arguments and set TAPPAS_FOUND to TRUE if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(TAPPAS DEFAULT_MSG TAPPAS_INCLUDE_DIR DEBUG_LIBRARY)

IF (TAPPAS_FOUND)
    SET(TAPPAS_INCLUDE_DIRS ${TAPPAS_INCLUDE_DIR})
ELSE (TAPPAS_FOUND)
    MESSAGE(WARNING "TAPPAS libraries not found!")
ENDIF (TAPPAS_FOUND)

MARK_AS_ADVANCED(TAPPAS_INCLUDE_DIR TAPPAS_POSTPROC_LIB_DIR)

##EOF