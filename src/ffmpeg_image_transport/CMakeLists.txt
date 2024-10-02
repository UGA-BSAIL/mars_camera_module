cmake_minimum_required(VERSION 2.8)
project(ffmpeg_image_transport)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport ffmpeg_image_transport_msgs sensor_msgs dynamic_reconfigure)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(V4L2 REQUIRED)

generate_dynamic_reconfigure_options(cfg/EncoderDyn.cfg)

set(FFMPEG_FIND_REQUIRED TRUE)
include(cmake/ffmpeg.cmake)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp cv_bridge image_transport
ffmpeg_image_transport_msgs sensor_msgs dynamic_reconfigure)

find_package(OpenCV)

include_directories(BEFORE
  include
  ${FFMPEG_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

# add the plugin
add_library(${PROJECT_NAME} src/manifest.cpp src/ffmpeg_publisher.cpp
		src/ffmpeg_subscriber.cpp src/ffmpeg_encoder.cpp
		src/ffmpeg_decoder.cpp src/tdiff.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${FFMPEG_LIBRARIES} ${V4L2_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
		ffmpeg_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
