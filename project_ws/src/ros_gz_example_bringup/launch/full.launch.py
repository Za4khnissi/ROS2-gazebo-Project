import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    config = os.path.join(
        get_package_share_directory("multirobot_map_merge"), "config", "params.yaml"
    )

    # Load the SDF files
    sdf_file_3 = os.path.join(pkg_project_description, 'models', 'limo_105_3', 'model.sdf')
    with open(sdf_file_3, 'r') as infp:
        robot_105_3_desc = infp.read()

    sdf_file_4 = os.path.join(pkg_project_description, 'models', 'limo_105_4', 'model.sdf')
    with open(sdf_file_4, 'r') as infp:
        robot_105_4_desc = infp.read()

    # First group of nodes (Gazebo, bridge, state publishers)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'diff_drive.sdf'
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
                    'scan_topic': "/limo_105_3/scan"
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
                    "return_to_init": False,
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
                    "return_to_init": False,
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

    return LaunchDescription([
        bridge,
        gz_sim,
        robot_state_publisher_3,
        robot_state_publisher_4,
        identify_service_3,
        mission_service_3,
        identify_service_4,
        mission_service_4,
        slam_nodes,
        nav2_nodes,
        explore_nodes
    ])