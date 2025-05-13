#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Get the serbot_navigation directory
    serbot_navigation_dir = get_package_share_directory('serbot_navigation')
    
    # Set map directory with absolute path
    # Set map directory in a user-independent way
    ws_dir = os.path.expanduser(os.path.join('~', 'ros2_ws'))
    map_dir = os.path.join(ws_dir, 'src', 'maps')
    map_yaml_path = os.path.join(map_dir, 'serbot_map.yaml')
    
    
    # Get the parameter files
    controller_params_file = os.path.join(serbot_navigation_dir, 'config', 'navigation', 'controller_params_fast.yaml')
    planner_params_file = os.path.join(serbot_navigation_dir, 'config', 'navigation', 'planner_params_fast.yaml')
    bt_navigator_params_file = os.path.join(serbot_navigation_dir, 'config', 'behavior_tree_params.yaml')
    advanced_amcl_params_file = os.path.join(serbot_navigation_dir, 'config', 'advanced_amcl_params.yaml')
    
    # Launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    # Declare the launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_path,
        description='Full path to map yaml file to load')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    # Start the nav2_map_server with explicit parameters
    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': use_sim_time,
            'topic_name': 'map',
            'frame_id': 'map',
            # Explicitly set map parameters to match yaml file
            'resolution': 0.05,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
            'mode': 'trinary'
        }])
                    
    # Start the lifecycle manager for the map server
    start_lifecycle_manager_map_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }])
    
    # Start AMCL for localization with explicit parameters
    start_amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0,
            'transform_tolerance': 0.5,
            'publish_tf': True,
            'scan_topic': '/scan',
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'min_particles': 500,
            'max_particles': 2000,
            'kld_err': 0.01,
            'kld_z': 0.99,
            'update_min_d': 0.2,
            'update_min_a': 0.2,
            'resample_interval': 1,
            'laser_model_type': 'likelihood_field',
            'laser_max_range': 12.0,
            'laser_min_range': 0.1,
            'laser_z_hit': 0.95,
            'laser_z_short': 0.1,
            'laser_z_max': 0.05,
            'laser_z_rand': 0.05,
            'laser_sigma_hit': 0.2,
            'laser_lambda_short': 0.1,
            'odom_model_type': 'diff',
            'odom_alpha1': 0.1,
            'odom_alpha2': 0.1,
            'odom_alpha3': 0.05,
            'odom_alpha4': 0.05,
            'odom_alpha5': 0.0
        }])
    
    # Add lifecycle manager for AMCL
    start_lifecycle_manager_localization_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl']
        }])
    
    # Include the Nav2 launch file with explicit params
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map_subscribe_transient_local': 'true',  # Important for map loading
            'cmd_vel_topic': '/cmd_vel',
            'controller_params_file': controller_params_file,
            'planner_params_file': planner_params_file,
            'bt_navigator_params_file': bt_navigator_params_file
            # Removed params_file to avoid conflicts with the separately launched AMCL
        }.items())
    
    # Start RViz with the AMCL-specific navigation configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(serbot_navigation_dir, 'config', 'amcl_nav_config.rviz')],
        output='screen')
    
    # Odometry republisher node
    odometry_handler_node = Node(
        package='serbot_navigation',
        executable='odometry_handler',
        name='odometry_handler',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_input_topic': '/odom',
            'odom_output_topic': '/odom_corrected',
            'initialpose_topic': '/initialpose',
            'publish_tf': True
        }]
    )
    
    # Include the static transforms launch file
    static_transforms_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(serbot_navigation_dir, 'launch', 'static_transforms.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items())
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # First start the map server and its lifecycle manager
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_map_cmd)
    
    # Start static transforms before navigation components
    ld.add_action(static_transforms_launch)
    
    # Then start navigation components
    ld.add_action(nav2_launch)
    
    # Then start AMCL and its lifecycle manager
    ld.add_action(start_amcl_cmd)
    ld.add_action(start_lifecycle_manager_localization_cmd)
    ld.add_action(odometry_handler_node)
    
    # Finally start RViz
    ld.add_action(rviz_node)
    
    return ld
