cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

find_package(Boost REQUIRED COMPONENTS program_options)

add_custom_target(VersionCpp ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_SOURCE_DIR} -P ${CMAKE_CURRENT_LIST_DIR}/version.cmake)
set_source_files_properties(version.cpp PROPERTIES GENERATED 1)

add_library(libcamera_app rpicam_app.cpp post_processor.cpp version.cpp options.cpp buffer_sync.cpp dma_heaps.cpp)
add_dependencies(libcamera_app VersionCpp)
target_include_directories(libcamera_app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_link_libraries(libcamera_app preview encoders)
target_compile_features(libcamera_app PUBLIC cxx_std_17)

target_link_libraries(libcamera_app pthread preview ${LIBCAMERA_LINK_LIBRARIES} ${Boost_LIBRARIES} post_processing_stages)

install(TARGETS libcamera_app LIBRARY DESTINATION lib/${PROJECT_NAME} ARCHIVE DESTINATION lib/${PROJECT_NAME})

list(APPEND ${PROJECT_NAME}_HEADERS
    completed_request.hpp
    frame_info.hpp
    rpicam_app.hpp
    rpicam_encoder.hpp
    logging.hpp
    metadata.hpp
    options.hpp
    post_processor.hpp
    still_options.hpp
    stream_info.hpp
    version.hpp
    video_options.hpp
    buffer_sync.hpp
    dma_heaps.hpp
)

install(FILES
    ${${PROJECT_NAME}_HEADERS}
    DESTINATION
    include/${PROJECT_NAME}/core
    COMPONENT Devel
)
