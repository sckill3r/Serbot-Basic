#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the slam_toolbox directory
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Include the slam_toolbox online_async launch file
    # Use local SLAM config file
    local_slam_config = os.path.join(
        get_package_share_directory('serbot_slam'), 'config', 'mapper_params_online_async.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': local_slam_config
        }.items()
    )
    
    # RViz with your existing configuration
    # Set RViz config path in a user-independent way
    ws_dir = os.path.expanduser(os.path.join('~', 'ros2_ws'))
    rviz_config_path = os.path.join(ws_dir, 'src', 'viz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    
    # Create and return launch description
    return LaunchDescription([
        slam_launch,
        rviz_node
    ])
