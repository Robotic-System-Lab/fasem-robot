#!/usr/bin/env python3
"""
Example: How to integrate nav_frontier.launch.py with custom cmd_vel topic
This example shows different ways to use the dynamic cmd_vel topic feature
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Example launch file showing integration of nav_frontier.launch.py
    with custom cmd_vel topic remapping for FASEM robot
    """
    
    # Declare arguments for this launch file
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (true) or real robot time (false)'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/a200_1060/cmd_vel',
        description='Custom cmd_vel topic for FASEM robot'
    )
    
    # Get the path to frontier_exp_cpp package
    frontier_pkg_dir = get_package_share_directory('frontier_exp_cpp')
    
    # Include nav_frontier.launch.py with custom cmd_vel topic
    nav_frontier_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(frontier_pkg_dir, 'launch', 'nav_frontier.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')
        }.items()
    )
    
    return LaunchDescription([
        # Declare arguments
        use_sim_time_arg,
        cmd_vel_topic_arg,
        
        # Launch navigation with frontier exploration
        nav_frontier_launch,
    ])


"""
USAGE EXAMPLES:

1. Default FASEM cmd_vel topic:
   ros2 launch <your_package> fasem_frontier_example.launch.py

2. Custom cmd_vel topic:
   ros2 launch <your_package> fasem_frontier_example.launch.py cmd_vel_topic:=/my_robot/cmd_vel

3. Real robot mode:
   ros2 launch <your_package> fasem_frontier_example.launch.py use_sim_time:=false

4. Specific configuration:
   ros2 launch <your_package> fasem_frontier_example.launch.py \\
       use_sim_time:=true \\
       cmd_vel_topic:=/fasem/velocity_commands
"""
