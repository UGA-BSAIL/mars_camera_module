# toplevel CMakeLists.txt for a ROS2 workspace

cmake_minimum_required(VERSION 3.5)  # ROS2 requires at least CMake 3.5
project(mars_camera_module)

# Boost features require >C++14
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include("${PROJECT_SOURCE_DIR}/scripts/cmake/colcon-toplevel-cmake/colcon.cmake")

# only for clion highlighting and analysis
colcon_add_subdirectories(
        BUILD_BASE "${PROJECT_SOURCE_DIR}/build"
        BASE_PATHS "${PROJECT_SOURCE_DIR}/src"
        # --packages-select
)