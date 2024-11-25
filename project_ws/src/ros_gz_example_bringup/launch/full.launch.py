#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def setup_robot_descriptions(context):
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    config = os.path.join(
        get_package_share_directory("multirobot_map_merge"), "config", "params.yaml"
    )

    # Get the drive mode configurations
    drive_mode_3 = LaunchConfiguration('drive_mode_3').perform(context)
    drive_mode_4 = LaunchConfiguration('drive_mode_4').perform(context)

    def get_robot_model_path(robot_number, drive_mode):
        return os.path.join(
            get_package_share_directory('ros_gz_example_description'),
            'models',
            f'limo_105_{robot_number}_{drive_mode}',
            'model.sdf'
        )

    # Retrieve paths based on the drive modes
    sdf_file_3 = get_robot_model_path(3, drive_mode_3)
    sdf_file_4 = get_robot_model_path(4, drive_mode_4)

    # Load the SDF files for Robot 3 and 4
    with open(sdf_file_3, 'r') as infp:
        robot_105_3_desc = infp.read()

    with open(sdf_file_4, 'r') as infp:
        robot_105_4_desc = infp.read()

    # First group of nodes (Gazebo, bridge, state publishers)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'modified_world.sdf'
        ])]}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    robot_state_publisher_3 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='/limo_105_3',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_105_3_desc},
        ]
    )

    robot_state_publisher_4 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='/limo_105_4',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_105_4_desc},
        ]
    )

    identify_service_3 = Node(
        package='ros_gz_example_application',
        executable='identify_service.py',
        name='identify_service',
        output='screen',
        namespace='/limo_105_3'
    )

    mission_service_3 = Node(
        package='ros_gz_example_application',
        executable='mission_service.py',
        name='mission_service',
        output='screen',
        namespace='/limo_105_3'
    )

    identify_service_4 = Node(
        package='ros_gz_example_application',
        executable='identify_service.py',
        name='identify_service',
        output='screen',
        namespace='/limo_105_4'
    )

    mission_service_4 = Node(
        package='ros_gz_example_application',
        executable='mission_service.py',
        name='mission_service',
        output='screen',
        namespace='/limo_105_4'
    )

    battery_node_3 = Node(
        package='ros_gz_example_application',
        executable='battery_manager.py',
        name='battery_node',
        output='screen',
        namespace='limo_105_3',
        parameters=[{'is_simulation': True}]
    )

    battery_node_4 = Node(
        package='ros_gz_example_application',
        executable='battery_manager.py',
        name='battery_node',
        output='screen',
        namespace='limo_105_4',
        parameters=[{'is_simulation': True}]
    )

    octomap_node = TimerAction(
        period=5.0,
        actions=[
            Node(package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[os.path.join(pkg_project_bringup, 'config', 'octomap_params.yaml')],
            remappings=[
                ('/cloud_in', '/limo_105_3/depth/image_raw/points'),  # Point to your point cloud topic
            ],
        )
        ]
    )


    # Second group of nodes (SLAM) - Delayed by 5 seconds
    slam_nodes = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox_1',
                namespace='limo_105_3',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'map_frame': 'map',
                    'odom_frame': "limo_105_3/odom",
                    'base_frame': "limo_105_3/base_footprint",
                    'scan_topic': "/limo_105_3/scan",
                    'max_laser_range': 8.0,
                    'resolution': 0.05,
                    'minimum_travel_distance': 0.5,
                    'minimum_travel_heading': 0.2,
                    'use_scan_matching': True,
                    'loop_closure_frequency': 1.0,
                    'map_update_interval': 2.0
                }],
                remappings=[
                    ('/map', '/limo_105_3/map'),
                    ('/map_metadata', '/limo_105_3/map_metadata'),
                    ('/scan', '/limo_105_3/scan'),
                    ('/tf', '/tf'),
                    ('/tf_static', '/tf_static'),
                    ('/odom', '/limo_105_3/odom')
                ],
            ),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox_2',
                namespace='limo_105_4',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'map_frame': 'map',
                    'odom_frame': "limo_105_4/odom",
                    'base_frame': "limo_105_4/base_footprint",
                    'scan_topic': "/limo_105_4/scan"
                }],
                remappings=[
                    ('/map', '/limo_105_4/map'),
                    ('/map_metadata', '/limo_105_4/map_metadata'),
                    ('/scan', '/limo_105_4/scan'),
                    ('/tf', '/tf'),
                    ('/tf_static', '/tf_static'),
                    ('/odom', '/limo_105_4/odom')
                ],
            ),
        ]
    )

    # Third group of nodes (Nav2) - Delayed by 10 seconds
    nav2_nodes = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_project_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'namespace': 'limo_105_3',
                    'use_namespace': 'True',
                    'use_sim_time': 'True',
                    'params_file': os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml'),
                    'map': os.path.join(pkg_project_bringup, 'config', 'sim_map.yaml'),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_project_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'namespace': 'limo_105_4',
                    'use_namespace': 'True',
                    'use_sim_time': 'True',
                    'params_file': os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml'),
                    'map': os.path.join(pkg_project_bringup, 'config', 'sim_map.yaml'),
                }.items(),
            ),
        ]
    )

    # Fourth group of nodes (Explore Lite) - Delayed by 15 seconds
    explore_nodes = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="explore_lite",
                name="explore_node",
                executable="explore",
                namespace="limo_105_3",
                parameters=[{
                    "robot_base_frame": "limo_105_3",
                    "return_to_init": True,
                    "costmap_topic": "/limo_105_3/global_costmap/costmap",
                    "visualize": False,
                    "planner_frequency": 0.15,
                    "progress_timeout": 30.0,
                    "potential_scale": 10.0,
                    "orientation_scale": 0.1,
                    "gain_scale": 1.0,
                    "transform_tolerance": 0.3,
                    "min_frontier_size": 0.02,
                    "use_sim_time": True
                }],
                output="screen",
            ),
            Node(
                package="explore_lite",
                name="explore_node_2",
                executable="explore",
                namespace="limo_105_4",
                parameters=[{
                    "robot_base_frame": "limo_105_4",
                    "return_to_init": True,
                    "costmap_topic": "/limo_105_4/global_costmap/costmap",
                    "visualize": False,
                    "planner_frequency": 0.15,
                    "progress_timeout": 30.0,
                    "potential_scale": 10.0,
                    "orientation_scale": 0.1,
                    "gain_scale": 1.0,
                    "transform_tolerance": 0.3,
                    "min_frontier_size": 0.02,
                    "use_sim_time": True
                }],
                output="screen",
            ),
            Node(
                package="multirobot_map_merge",
                name="map_merge",
                #namespace=namespace,
                executable="map_merge",
                parameters=[
                    config,
                    {"use_sim_time": True},
                    {"known_init_poses": True},
                ],
                output="screen",
                #remappings=remappings,
            )
        ]
    )

    random_walker_nodes = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='random_walker',
                executable='random_walker_node',
                name='random_walker',
                output='screen',
                namespace='/limo_105_3',
                parameters=[{
                    'min_x': -5.0,
                    'max_x': 5.0,
                    'min_y': -5.0,
                    'max_y': 5.0,
                    'map_frame': "map",
                    'robot_frame': f"limo_105_3/base_footprint",
                    'nav_action_server': f"/limo_105_3/navigate_to_pose",
                    'goal_tolerance': 0.5,
                    'min_distance_from_current': 0.1,
                    'max_attempts': 3,
                    'timeout_delay': 100.0,
                    'return_to_init': True,
                    'progress_timeout': 30.0,
                    'pose_topic': f"/limo_inf3995105_3/amcl_pose"
                }]
            )
        ]
    )

    random_walker_nodes = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='random_walker',
                executable='random_walker_node',
                name='random_walker',
                output='screen',
                namespace='/limo_105_3',
                parameters=[{
                    'min_x': -5.0,
                    'max_x': 5.0,
                    'min_y': -5.0,
                    'max_y': 5.0,
                    'map_frame': "map",
                    'robot_frame': f"limo_105_3/base_footprint",
                    'nav_action_server': f"/limo_105_3/navigate_to_pose",
                    'goal_tolerance': 0.5,
                    'min_distance_from_current': 0.1,
                    'max_attempts': 3,
                    'timeout_delay': 100.0,
                    'return_to_init': True,
                    'progress_timeout': 30.0,
                    'pose_topic': f"/limo_inf3995105_3/amcl_pose"
                }]
            )
        ]
    )

    return [
        gz_sim,
        bridge,
        identify_service_3,
        mission_service_3,
        identify_service_4,
        mission_service_4,
        battery_node_3,
        battery_node_4,
        robot_state_publisher_3,
        robot_state_publisher_4,
        octomap_node,
        slam_nodes,
        nav2_nodes,
        explore_nodes,
        random_walker_nodes
    ]

def generate_launch_description():
    # Declare launch arguments for drive modes
    drive_mode_3_arg = DeclareLaunchArgument(
        'drive_mode_3',
        default_value='diff_drive',
        description='Drive mode for robot 3'
    )
    drive_mode_4_arg = DeclareLaunchArgument(
        'drive_mode_4',
        default_value='diff_drive',
        description='Drive mode for robot 4'
    )

    return LaunchDescription([
        drive_mode_3_arg,
        drive_mode_4_arg,
        OpaqueFunction(function=setup_robot_descriptions),
    ])
