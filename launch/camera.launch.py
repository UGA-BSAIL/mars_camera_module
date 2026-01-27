from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_lib_directory
import os


def generate_camera_node(*, device_id: int, quality: int, postprocess_file: LaunchConfiguration):
    libcamera_device_dir = get_package_lib_directory('libcamera_device')
    return LaunchDescription([
        Node(
            package='libcamera_device',
            executable=os.path.join(libcamera_device_dir, 'libcamera_device_node'),
            # prefix=['gdbserver localhost:3000'],
            name=LaunchConfiguration('node_name'),
            output='screen',
            # This ensures the whole launch will shut down if this node exits
            on_exit=[Shutdown()],
            parameters=[{
                'ffmpeg/qmax': str(quality),
                'ffmpeg/encoder': LaunchConfiguration('encoder'),
                'ffmpeg/profile': 'baseline',
                'ffmpeg/bit_rate': 16000000,
                'fps': '24',
                'width': 1920,
                'height': 1080,
                'frame_id': LaunchConfiguration('frame_id'),
                'device_id': str(device_id),
                'postprocess_file': postprocess_file,
            }],
            remappings=[
                ('detections', ['/', LaunchConfiguration('node_name'), '/detections']),
                ('motion', ['/', LaunchConfiguration('node_name'), '/motion']),
            ],
        )
    ])


def generate_launch_description():
    node_name_arg = DeclareLaunchArgument(
        "node_name", default_value=TextSubstitution(text="camera"),
        description="The name to use for the camera node."
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id", default_value=TextSubstitution(text="frame"),
        description="The frame ID to use for the camera."
    )
    encoder_arg = DeclareLaunchArgument(
        "encoder", default_value=TextSubstitution(text="h264_v4l2m2m"),
        description="The ffmpeg encoder name to use."
    )
    postprocess_file_arg = DeclareLaunchArgument(
        "postprocess_file",
        default_value=TextSubstitution(
            text="/usr/share/rpi-camera-assets/ai_pheno/hailo_yolov8_flower_inference.json"
        ),
        description="The file defining the post-processing configuration to use."
    )
    ir_postprocess_file_arg = DeclareLaunchArgument(
        "ir_postprocess_file",
        default_value=TextSubstitution(
            text="/usr/share/rpi-camera-assets/ai_pheno/motion.json"
        ),
        description="IR camera post-processing config file."
    )

    # Get package share directories
    mars_camera_hw_manager_dir = get_package_share_directory('mars_camera_hw_manager')

    rgb_node = generate_camera_node(device_id=0, quality=4, postprocess_file=LaunchConfiguration('postprocess_file'))
    ir_node = generate_camera_node(device_id=1, quality=8, postprocess_file=LaunchConfiguration('ir_postprocess_file'))

    cam_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mars_camera_hw_manager_dir, 'launch', 'manager.launch.py')
        ),
        launch_arguments={
            'camera_id': LaunchConfiguration('node_name')
        }.items()
    )

    return LaunchDescription([
        node_name_arg,
        frame_id_arg,
        encoder_arg,
        postprocess_file_arg,
        ir_postprocess_file_arg,
        rgb_node,
        ir_node,
        cam_manager_launch
    ])
