#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('final_project_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    map_file = os.path.join(pkg_share, 'maps', 'my_navigation_map.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    
    # Launch configurations
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_namespace = LaunchConfiguration('use_namespace')
    
    # Declare arguments
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Auto-start Nav2 stack')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=map_file,
        description='Full path to map file')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=nav2_params,
        description='Full path to Nav2 params file')
    
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false',
        description='Use namespace')
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Respawn if node crashes')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')
    
    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items()
    )
    
    return LaunchDescription([
        declare_autostart_cmd,
        declare_map_yaml_cmd,
        declare_namespace_cmd,
        declare_params_file_cmd,
        declare_use_composition_cmd,
        declare_use_namespace_cmd,
        declare_use_respawn_cmd,
        declare_use_sim_time_cmd,
        nav2_bringup
    ])
