cmake_minimum_required(VERSION 3.0.2)

add_subdirectory(core)
add_subdirectory(encoder)
add_subdirectory(preview)
add_subdirectory(image)
add_subdirectory(post_processing_stages)

set(CMAKE_CXX_FLAGS_DEBUG "-g")

find_package(PkgConfig REQUIRED)
pkg_check_modules(VALGRIND REQUIRED valgrind)

# Add node executable.
add_executable(${PROJECT_NAME}_node camera_node.cpp camera_messenger.cpp)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${LIBCAMERA_LIBRARIES}
        libcamera_app)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
target_include_directories(${PROJECT_NAME}_node PRIVATE ${VALGRIND_INCLUDE_DIRS})