cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

find_library(EXIF_LIBRARY exif REQUIRED)
find_library(JPEG_LIBRARY jpeg REQUIRED)
find_library(TIFF_LIBRARY tiff REQUIRED)
find_library(PNG_LIBRARY png REQUIRED)

add_library(images bmp.cpp yuv.cpp jpeg.cpp png.cpp dng.cpp)
target_link_libraries(images jpeg exif png tiff)
target_compile_features(images PRIVATE cxx_std_17)

install(TARGETS images LIBRARY DESTINATION lib/${PROJECT_NAME} ARCHIVE DESTINATION lib/${PROJECT_NAME})

list(APPEND ${PROJECT_NAME}_HEADERS
    image.hpp
)

install(FILES
    ${${PROJECT_NAME}_HEADERS}
    DESTINATION
    include/${PROJECT_NAME}/image
    COMPONENT Devel
)

