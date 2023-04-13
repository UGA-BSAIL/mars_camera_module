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
  # Check if building on Nvidia hardware.
  IF(EXISTS "/dev/nvhost-nvdec")
    SET(NVIDIA_BUILD TRUE)
    message(STATUS "Building for an Nvidia system.")
  ELSE()
    SET(NVIDIA_BUILD FALSE)
    message(STATUS "Building for a non-Nvidia system.")
  ENDIF()

  include(ProcessorCount)
  ProcessorCount(NUM_CORES)

  IF(${NVIDIA_BUILD})
    # Install FFMPEG external dependency.
    SET(FFMPEG_SRC "ffmpeg/src/ffmpeg")
    include(ExternalProject)
    ExternalProject_Add(ffmpeg
            PREFIX "ffmpeg"
            DOWNLOAD_COMMAND bash -c "apt-get source ffmpeg && mv ffmpeg-4.2.7/* ffmpeg/"
            CONFIGURE_COMMAND cp ${CMAKE_CURRENT_LIST_DIR}/configure_ffmpeg.sh . && ./configure_ffmpeg.sh "${CMAKE_CURRENT_LIST_DIR}/.."
            BUILD_COMMAND make -j${NUM_CORES}
            BUILD_IN_SOURCE TRUE)

    SET(FFMPEG_INSTALL_DIR "${CMAKE_CURRENT_LIST_DIR}/../external")
    SET(FFMPEG_AVCODEC_INCLUDE_DIR "${FFMPEG_INSTALL_DIR}/include")
    SET(FFMPEG_LIBSWSCALE "${FFMPEG_INSTALL_DIR}/lib/libswscale.so.5")
    SET(FFMPEG_LIBSWRESAMPLE "${FFMPEG_INSTALL_DIR}/lib/libswresample.so.3")
    SET(FFMPEG_LIBAVCODEC "${FFMPEG_INSTALL_DIR}/lib/libavcodec.so.58")
    SET(FFMPEG_LIBAVFORMAT "${FFMPEG_INSTALL_DIR}/lib/libavformat.so.58")
    SET(FFMPEG_LIBAVUTIL "${FFMPEG_INSTALL_DIR}/lib/libavutil.so.56")
  ELSE()
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
  ENDIF()


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

