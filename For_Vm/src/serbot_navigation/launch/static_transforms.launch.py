#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Static transform publishers for the robot's TF tree
    
    # Map to Odom transform (initially identity, will be updated by AMCL)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # Base link to base footprint transform
    static_tf_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )
    
    # Base link to laser frame transform
    static_tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the nodes to the launch description
    ld.add_action(static_tf_map_odom)
    ld.add_action(static_tf_base_footprint)
    ld.add_action(static_tf_base_laser)
    
    return ld 