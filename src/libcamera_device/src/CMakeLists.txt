cmake_minimum_required(VERSION 3.5)

add_subdirectory(core)
add_subdirectory(encoder)
add_subdirectory(preview)
add_subdirectory(image)
add_subdirectory(post_processing_stages)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(libcamera_device_msgs REQUIRED)

set(CMAKE_CXX_FLAGS_DEBUG "-g")

# Add node executable.
add_executable(${PROJECT_NAME}_node camera_node.cpp camera_messenger.cpp)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_20)
target_link_libraries(${PROJECT_NAME}_node
        ${LIBCAMERA_LIBRARIES}
        libcamera_app
        hailo-postproc)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp image_transport sensor_msgs libcamera_device_msgs)

install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)