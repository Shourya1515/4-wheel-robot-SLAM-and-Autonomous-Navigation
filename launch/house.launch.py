#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # IMPORTANT: Add this
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'final_project_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    world_file = os.path.join(pkg_share, 'worlds', 'house_world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'final_project.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'gazebo_bridge.yaml')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    
    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')
    
    declare_x_cmd = DeclareLaunchArgument('x', default_value='-3.5')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='-3.5')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.5')
    declare_roll_cmd = DeclareLaunchArgument('roll', default_value='0.0')
    declare_pitch_cmd = DeclareLaunchArgument('pitch', default_value='0.0')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0')
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )
    
    # Robot description - FIXED with ParameterValue
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'final_project',
            '-x', x, '-y', y, '-z', z,
            '-R', roll, '-P', pitch, '-Y', yaw
        ],
        output='screen'
    )
    
    # Bridge using YAML config
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}'
        ],
        output='screen'
    )
    
    # Odom to TF publisher
    odom_to_tf = Node(
        package='final_project_description',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_x_cmd, declare_y_cmd, declare_z_cmd,
        declare_roll_cmd, declare_pitch_cmd, declare_yaw_cmd,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        odom_to_tf
    ])
