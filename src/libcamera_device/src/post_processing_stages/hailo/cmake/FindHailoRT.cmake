# - Find Hailort library
# Find the native Hailort includes and library
# This module defines
#  HAILORT_INCLUDE_DIR, where to find libhailort.h, etc.
#  HAILORT_LIBRARIES, libraries to link against to use Hailort.
#  HAILORT_FOUND, If false, do not try to use Hailort. also defined, but not for general use are
#  HAILORT_LIBRARY, where to find the Hailort library.
#

SET (HAILORT_FOUND 0)

FIND_PATH (HAILORT_INCLUDE_DIR
        NAMES hailort.hpp
        PATH_SUFFIXES hailo
        DOC "The HailoRT include directory"
)

FIND_LIBRARY (HAILORT_LIBRARY
        NAMES hailort
        DOC "The HailoRT library"
)

# handle the QUIETLY and REQUIRED arguments and set HAILORT_FOUND to TRUE if all listed variables are TRUE
INCLUDE (FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS (HAILORT DEFAULT_MSG HAILORT_LIBRARY HAILORT_INCLUDE_DIR)

IF (HAILORT_FOUND)
    SET (HAILORT_LIBRARIES ${HAILORT_LIBRARY})
    SET (HAILORT_INCLUDE_DIRS ${HAILORT_INCLUDE_DIR})
ELSE (HAILORT_FOUND)
    MESSAGE (WARNING "libhailort library not found!")
ENDIF (HAILORT_FOUND)

MARK_AS_ADVANCED (HAILORT_INCLUDE_DIR HAILORT_LIBRARY)
##EOF