from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_id_arg = DeclareLaunchArgument(
        'manager_name',
        default_value='camera'
    )

    manager_name = LaunchConfiguration('manager_name')

    # Hardware monitor node. It needs to be running for fan control to work.
    hw_monitor_node = Node(
        package='ros_hw_monitor',
        executable='hw_monitor',
        name=[
            manager_name, '_hw_monitor'
        ],
        output='screen',
        remappings=[
            ('system_info', ['/', manager_name, '/system_info'])
        ]
    )

    fan_control_node = Node(
        package='mars_camera_hw_manager',
        executable='fan_controller',
        name=['camera_fan_control_', manager_name],
        parameters=[{
            # Fan is connected to pin 12 on v2 and stereo modules.
            "fan_pin": 12,
        }],
        remappings=[
            ('system_info', ['/', manager_name, '/system_info'])
        ]
    )

    manager_node = Node(
        package='mars_camera_hw_manager',
        executable='manager',
        name=['camera_hw_manager_', manager_name],
        remappings=[
            ('control', ['/', manager_name, '/control'])
        ]
    )

    hardware_info_node = Node(
        package='mars_camera_hw_manager',
        executable='hardware_info',
        name=['camera_hw_info_', manager_name],
        output='screen',
        remappings=[
            ('camera_info', ['/', manager_name, '/hw_config'])
        ]
    )

    return LaunchDescription([
        camera_id_arg,
        hw_monitor_node,
        fan_control_node,
        manager_node,
        hardware_info_node,
    ])
