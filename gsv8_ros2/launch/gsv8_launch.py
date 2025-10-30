#!/usr/bin/env python3
"""
Launch file for GSV8 force-torque sensor publisher
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for GSV8 publisher"""

    # Declare launch arguments
    adapter_arg = DeclareLaunchArgument(
        'adapter_name',
        default_value='',
        description='EtherCAT adapter name (e.g., \\Device\\NPF_{GUID} on Windows, eth0 on Linux)'
    )

    slave_position_arg = DeclareLaunchArgument(
        'slave_position',
        default_value='0',
        description='EtherCAT slave position (default: 0 for first slave)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Publishing rate in Hz (default: 100.0)'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='ft_sensor',
        description='Frame ID for the sensor (default: ft_sensor)'
    )

    publish_all_channels_arg = DeclareLaunchArgument(
        'publish_all_channels',
        default_value='true',
        description='Publish all 8 channels as Float32MultiArray (default: true)'
    )

    # Create node
    gsv8_publisher_node = Node(
        package='gsv8_ros2',
        executable='gsv8_publisher',
        name='gsv8_publisher',
        output='screen',
        parameters=[{
            'adapter_name': LaunchConfiguration('adapter_name'),
            'slave_position': LaunchConfiguration('slave_position'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_all_channels': LaunchConfiguration('publish_all_channels'),
        }]
    )

    return LaunchDescription([
        adapter_arg,
        slave_position_arg,
        publish_rate_arg,
        frame_id_arg,
        publish_all_channels_arg,
        gsv8_publisher_node,
    ])
