#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    
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
    
    # RViz2 node WITHOUT config file to avoid class loading errors
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        exploration_node,
        rviz_node
    ])