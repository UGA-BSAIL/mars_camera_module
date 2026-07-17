import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value=TextSubstitution(text="camera"),
        description="The name to use for the camera node.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value=TextSubstitution(text="frame"),
        description="The frame ID to use for the camera.",
    )
    encoder_arg = DeclareLaunchArgument(
        "encoder",
        default_value=TextSubstitution(text="h264_v4l2m2m"),
        description="The ffmpeg encoder name to use.",
    )
    pixel_format_arg = DeclareLaunchArgument(
        "pixel_format",
        default_value=TextSubstitution(text="yuv420p"),
        description="The pixel format to use for the encoder.",
    )
    postprocess_file_arg = DeclareLaunchArgument(
        "postprocess_file",
        default_value=TextSubstitution(
            text="/usr/share/rpi-camera-assets/ai_pheno/hailo_yolov8_flower_inference.json"
        ),
        description="The file defining the post-processing configuration to use.",
    )
    ir_postprocess_file_arg = DeclareLaunchArgument(
        "ir_postprocess_file",
        default_value=TextSubstitution(
            text="/usr/share/rpi-camera-assets/ai_pheno/motion.json"
        ),
        description="IR camera post-processing config file.",
    )

    # Get package share directories
    libcamera_device_dir = get_package_share_directory("libcamera_device")
    mars_camera_hw_manager_dir = get_package_share_directory("mars_camera_hw_manager")

    rgb_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(libcamera_device_dir, "launch", "full_hd_video.launch.py")
        ),
        launch_arguments={
            "camera_name": LaunchConfiguration("node_name"),
            "frame_id": LaunchConfiguration("frame_id"),
            "device_id": "0",
            "quality": "4",
            "encoder": LaunchConfiguration("encoder"),
            "pixel_format": LaunchConfiguration("pixel_format"),
            "fps": "24",
            "postprocess_file": LaunchConfiguration("postprocess_file"),
        }.items(),
    )

    ir_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(libcamera_device_dir, "launch", "full_hd_video.launch.py")
        ),
        launch_arguments={
            "camera_name": [LaunchConfiguration("node_name"), "_ir"],
            "frame_id": [LaunchConfiguration("frame_id"), "_ir"],
            "device_id": "1",
            "quality": "8",
            "encoder": LaunchConfiguration("encoder"),
            "pixel_format": LaunchConfiguration("pixel_format"),
            "fps": "24",
            "postprocess_file": LaunchConfiguration("ir_postprocess_file"),
        }.items(),
    )

    cam_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mars_camera_hw_manager_dir, "launch", "manager.launch.py")
        ),
        launch_arguments={"manager_name": LaunchConfiguration("node_name")}.items(),
    )

    return LaunchDescription(
        [
            node_name_arg,
            frame_id_arg,
            encoder_arg,
            pixel_format_arg,
            postprocess_file_arg,
            ir_postprocess_file_arg,
            rgb_launch,
            ir_launch,
            cam_manager_launch,
        ]
    )
