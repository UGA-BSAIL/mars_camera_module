from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='camera'),
        DeclareLaunchArgument('frame_id', default_value='frame'),
        DeclareLaunchArgument('encoder', default_value='h264_v4l2m2m'),
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('quality', default_value='2'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('postprocess_file', default_value=''),

        Node(
            package='libcamera_device',
            executable='libcamera_device_node',
            # prefix=['gdbserver localhost:3000'],
            name=LaunchConfiguration('node_name'),
            output='screen',
            # This ensures the whole launch will shut down if this node exits
            on_exit=[Shutdown()],
            parameters=[{
                'ffmpeg/qmax': LaunchConfiguration('quality'),
                'ffmpeg/encoder': LaunchConfiguration('encoder'),
                'ffmpeg/profile': 'baseline',
                'ffmpeg/bit_rate': 16000000,
                'fps': LaunchConfiguration('fps'),
                'width': 1920,
                'height': 1080,
                'frame_id': LaunchConfiguration('frame_id'),
                'device_id': LaunchConfiguration('device_id'),
                'postprocess_file': LaunchConfiguration('postprocess_file'),
            }],
            remappings=[
                ('detections', [ '/', LaunchConfiguration('node_name'), '/detections' ]),
                ('motion', [ '/', LaunchConfiguration('node_name'), '/motion' ]),
            ],
        )
    ])
