#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('autonomous_exploration'),
            'config',
            'exploration_visualization.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Autonomous exploration node
    exploration_node = Node(
        package='autonomous_exploration',
        executable='control',
        name='autonomous_exploration_control',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_vel': 'cmd_vel',
            'lookahead_distance': 0.24,
            'speed': 0.12,
            'expansion_size': 20,
            'target_error': 0.3,
            'robot_r': 1.5,
            'max_angular_velocity': 0.4
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/map', '/map'),
            ('/fasem_odom', '/fasem_odom'),
            ('/fasem_scan', '/fasem_scan')
        ]
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        rviz_config_arg,
        use_sim_time_arg,
        exploration_node,
        rviz_node
    ])