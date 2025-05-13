#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription()
    
    # Create our own map directory
    map_save_dir = os.path.join(os.getcwd(), 'src', 'maps')
    os.makedirs(map_save_dir, exist_ok=True)
    
    # Declare the launch arguments
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='my_new_map',
        description='Name of the map to save'
    )
    
    # Add the map_saver node
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        arguments=[
            '-f', [map_save_dir, '/', LaunchConfiguration('map_name')],
            '--ros-args', '--log-level', 'info'
        ]
    )
    
    # Add the declared launch arguments to the launch description
    ld.add_action(map_name_arg)
    
    # Add nodes to the launch description
    ld.add_action(map_saver_node)
    
    return ld
