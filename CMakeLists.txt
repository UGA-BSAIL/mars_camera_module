cmake_minimum_required(VERSION 2.8)
project(ffmpeg_image_transport_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)
add_message_files(DIRECTORY msg FILES FFMPEGPacket.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

