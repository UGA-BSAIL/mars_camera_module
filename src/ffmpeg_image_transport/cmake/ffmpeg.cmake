# - Try to find ffmpeg libraries (libavcodec, libavformat and libavutil)
# Once done this will define
#
#  FFMPEG_FOUND - system has ffmpeg or libav
#  FFMPEG_INCLUDE_DIR - the ffmpeg include directory
#  FFMPEG_LIBRARIES - Link these to use ffmpeg
#  FFMPEG_LIBAVCODEC
#  FFMPEG_LIBAVFORMAT
#  FFMPEG_LIBAVUTIL
#  FFMPEG_LIBSWSCALE
#  FFMPEG_LIBSWRESAMPLE
#
#  Copyright (c) 2008 Andreas Schneider <mail@cynapses.org>
#  Modified for other libraries by Lasse Kärkkäinen <tronic>
#  Modified for Hedgewars by Stepik777
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#

function(set_ffmpeg_paths ffmpeg_lib ffmpeg_inc)
  find_path(FFMPEG_AVCODEC_INCLUDE_DIR
          NAMES libavcodec/avcodec.h
          HINTS ${ffmpeg_inc}
          PATH_SUFFIXES ffmpeg libav
          )

  find_library(FFMPEG_LIBAVCODEC
          NAMES avcodec
          HINTS ${ffmpeg_lib}
          )

  find_library(FFMPEG_LIBAVFORMAT
          NAMES avformat
          HINTS ${ffmpeg_lib}
          )

  find_library(FFMPEG_LIBAVUTIL
          NAMES avutil
          HINTS ${ffmpeg_lib}
          )

  find_library(FFMPEG_LIBSWSCALE
          NAMES swscale
          HINTS ${ffmpeg_lib}
          )

  find_library(FFMPEG_LIBSWRESAMPLE
          NAMES swresample
          HINTS ${ffmpeg_lib}
          )
endfunction()

if (FFMPEG_LIBRARIES AND FFMPEG_INCLUDE_DIR)
  # in cache already
  set(FFMPEG_FOUND TRUE)
else (FFMPEG_LIBRARIES AND FFMPEG_INCLUDE_DIR)
  # use pkg-config to get the directories and then use these values
  # in the FIND_PATH() and FIND_LIBRARY() calls
  find_package(PkgConfig)
  set(FFMPEG_EXTRA_LIB_DIRS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib /opt/local/lib /sw/lib)
  set(FFMPEG_EXTRA_LIB_DIRS ${FFMPEG_LIB} /usr/lib /usr/local/lib /opt/local/lib /sw/lib)
  if (PKG_CONFIG_FOUND)
    pkg_check_modules(_FFMPEG_AVCODEC libavcodec)
    pkg_check_modules(_FFMPEG_AVFORMAT libavformat)
    pkg_check_modules(_FFMPEG_AVUTIL libavutil)
    pkg_check_modules(_FFMPEG_SWSCALE libswscale)
    pkg_check_modules(_FFMPEG_SWRESAMPLE libswresample)
  endif (PKG_CONFIG_FOUND)

  set_ffmpeg_paths("${FFMPEC_LIB}" "${FFMPEG_INC}")

  if (FFMPEG_LIBAVCODEC AND FFMPEG_LIBAVFORMAT)
    set(FFMPEG_FOUND TRUE)
  endif()

  if (FFMPEG_FOUND)
    set(FFMPEG_INCLUDE_DIRS ${FFMPEG_AVCODEC_INCLUDE_DIR})

    set(FFMPEG_LIBRARIES
            ${FFMPEG_LIBSWSCALE}
            ${FFMPEG_LIBSWRESAMPLE}
            ${FFMPEG_LIBAVCODEC}
            ${FFMPEG_LIBAVFORMAT}
            ${FFMPEG_LIBAVUTIL}
            )

  endif (FFMPEG_FOUND)

  if (FFMPEG_FOUND)
    if (NOT FFMPEG_FIND_QUIETLY)
      message(STATUS "Found FFMPEG or Libav: ${FFMPEG_LIBRARIES}, ${FFMPEG_INCLUDE_DIR}")
    endif (NOT FFMPEG_FIND_QUIETLY)
  else (FFMPEG_FOUND)
    if (FFMPEG_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find libavcodec or libavformat or libavutil")
    endif (FFMPEG_FIND_REQUIRED)
  endif (FFMPEG_FOUND)

endif (FFMPEG_LIBRARIES AND FFMPEG_INCLUDE_DIR)

