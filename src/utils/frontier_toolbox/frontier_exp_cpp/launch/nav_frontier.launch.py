import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # Declare a launch argument for sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (true) or real robot time (false)'
    )

    # Declare a launch argument for cmd_vel topic
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='Topic name for velocity commands output'
    )

    # Get the path to the nav2_bringup package
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Get the path to the nav2_params.yaml in your package's launch folder
    package_share_dir = get_package_share_directory('frontier_exp_cpp')
    nav2_params_file_robot = os.path.join(
        package_share_dir, 'config', 'nav2_params_robot.yaml')
    frontier_params_file = os.path.join(
        get_package_share_directory('frontier_exp_cpp'),
        'config',
        'frontier_params.yaml'
    )

    # Function to launch Nav2 nodes with custom cmd_vel remapping
    def include_nav2_with_params(context):
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
        cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)
        
        # Convert use_sim_time string to boolean
        use_sim_time_bool = use_sim_time.lower() == 'true'
        
        configured_params = nav2_params_file_robot
        
        # Define lifecycle node names for lifecycle_manager
        lifecycle_nodes = ['controller_server',
                          'planner_server',
                          'behavior_server',
                          'bt_navigator',
                          'waypoint_follower',
                          'velocity_smoother']
        
        return [GroupAction([
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', cmd_vel_topic)],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', cmd_vel_topic)],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', cmd_vel_topic)],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time_bool,
                            'autostart': True,
                            'node_names': lifecycle_nodes}]
            ),
        ])]

    # Define the frontier exploration node
    frontier_node = LifecycleNode(
        package='frontier_exp_cpp',
        executable='frontier_lc',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    frontier_params_file],
        namespace=''
    )

    nav2_client_node = Node(
        package='nav_client_cpp',
        executable='nav_node',
        name='nav_to_pose',
        output='screen',
        parameters=[frontier_params_file])

    return LaunchDescription([
        # Declare launch arguments
        use_sim_time_arg,
        cmd_vel_topic_arg,

        # Conditionally launch Nav2 with or without params file
        OpaqueFunction(function=include_nav2_with_params),

        # Launch nav2 client node
        nav2_client_node,

        # Launch frontier node
        frontier_node,
    ])
