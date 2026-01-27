from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='camera'
    )

    camera_id = LaunchConfiguration('camera_id')

    manager_node = Node(
        package='mars_camera_hw_manager',
        executable='manager',
        name=['camera_hw_manager_', camera_id],
        remappings=[
            ('control', ['/', camera_id, '/control'])
        ]
    )

    hardware_info_node = Node(
        package='mars_camera_hw_manager',
        executable='hardware_info',
        name=['camera_hw_info_', camera_id],
        output='screen',
        remappings=[
            ('camera_info', ['/', camera_id, '/hw_config'])
        ]
    )

    return LaunchDescription([
        camera_id_arg,
        manager_node,
        hardware_info_node
    ])
