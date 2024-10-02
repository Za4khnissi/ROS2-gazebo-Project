#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext

def generate_launch_description():
    # Arguments to choose the drive mode for each robot
    drive_mode_3 = LaunchConfiguration('drive_mode_3', default='diff_drive')
    drive_mode_4 = LaunchConfiguration('drive_mode_4', default='diff_drive')

    # Function to get the SDF path based on drive mode
    def get_robot_model_path(robot_number, drive_mode):
        return os.path.join(
            get_package_share_directory('ros_gz_example_description'),
            'models',
            f'limo_105_{robot_number}_{drive_mode}',
            'model.sdf'
        )

    # Create a LaunchContext to resolve LaunchConfiguration values
    lc = LaunchContext()

    # Get the resolved SDF paths for the robots
    sdf_file_3 = get_robot_model_path(3, drive_mode_3.perform(lc))
    sdf_file_4 = get_robot_model_path(4, drive_mode_4.perform(lc))

    # Load the SDF file for Robot 3
    with open(sdf_file_3, 'r') as infp:
        robot_105_3_desc = infp.read()

    # Load the SDF file for Robot 4
    with open(sdf_file_4, 'r') as infp:
        robot_105_4_desc = infp.read()

    # Robot 3 Nodes
    robot_3_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace='limo_105_3',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_105_3_desc},
        ]
    )

    # Robot 4 Nodes
    robot_4_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace='limo_105_4',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_105_4_desc},
        ]
    )

    # Identify and Mission services for Robots 3 and 4
    identify_service_3 = Node(
        package='ros_gz_example_application',
        executable='identify_service.py',
        name='identify_service',
        output='screen',
        namespace='limo_105_3'
    )

    mission_service_3 = Node(
        package='ros_gz_example_application',
        executable='mission_service.py',
        name='mission_service',
        output='screen',
        namespace='limo_105_3'
    )

    identify_service_4 = Node(
        package='ros_gz_example_application',
        executable='identify_service.py',
        name='identify_service',
        output='screen',
        namespace='limo_105_4'
    )

    mission_service_4 = Node(
        package='ros_gz_example_application',
        executable='mission_service.py',
        name='mission_service',
        output='screen',
        namespace='limo_105_4'
    )

    return LaunchDescription([
        DeclareLaunchArgument('drive_mode_3', default_value='diff_drive'),
        DeclareLaunchArgument('drive_mode_4', default_value='diff_drive'),
        robot_3_state_publisher,
        robot_4_state_publisher,
        identify_service_3,
        mission_service_3,
        identify_service_4,
        mission_service_4
    ])
