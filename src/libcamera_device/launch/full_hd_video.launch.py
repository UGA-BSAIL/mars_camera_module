from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('frame_id', default_value='frame'),
        DeclareLaunchArgument('encoder', default_value='h264_v4l2m2m'),
        DeclareLaunchArgument('pixel_format', default_value='yuv420p'),
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('quality', default_value='2'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('postprocess_file', default_value=''),

        Node(
            package='libcamera_device',
            executable='libcamera_device_node',
            # prefix=['gdbserver localhost:3000'],
            name=LaunchConfiguration('camera_name'),
            output='screen',
            # This ensures the whole launch will shut down if this node exits
            on_exit=[Shutdown()],
            parameters=[{
                (LaunchConfiguration('camera_name'), '.ffmpeg.qmax'): LaunchConfiguration('quality'),
                (LaunchConfiguration('camera_name'), '.ffmpeg.encoder'): LaunchConfiguration('encoder'),
                (LaunchConfiguration('camera_name'), '.ffmpeg.pixel_format'): LaunchConfiguration('pixel_format'),
                (LaunchConfiguration('camera_name'), '.ffmpeg.gop_size'): 10,
                (LaunchConfiguration('camera_name'), '.ffmpeg.bit_rate'): 8000000,
                (LaunchConfiguration('camera_name'), '.ffmpeg.encoder_measure_performance'): True,
                'fps': LaunchConfiguration('fps'),
                'width': 1920,
                'height': 1080,
                'frame_id': LaunchConfiguration('frame_id'),
                'device_id': LaunchConfiguration('device_id'),
                'postprocess_file': LaunchConfiguration('postprocess_file'),
            }],
            remappings=[
                ('detections', [ '/', LaunchConfiguration('camera_name'), '/detections' ]),
                ('motion', [ '/', LaunchConfiguration('camera_name'), '/motion' ]),
            ],
        )
    ])
