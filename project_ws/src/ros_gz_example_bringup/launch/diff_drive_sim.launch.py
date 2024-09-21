#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Setup project paths
    pkg_project_description = get_package_share_directory('ros_gz_example_description')

    # Load the SDF file for Robot 3
    sdf_file_3 = os.path.join(pkg_project_description, 'models', 'limo_105_3', 'model.sdf')
    with open(sdf_file_3, 'r') as infp:
        robot_105_3_desc = infp.read()

    # Load the SDF file for Robot 4
    sdf_file_4 = os.path.join(pkg_project_description, 'models', 'limo_105_4', 'model.sdf')
    with open(sdf_file_4, 'r') as infp:
        robot_105_4_desc = infp.read()

    # Robot 3 Nodes with a unique namespace
    robot_3_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace='/limo_105_3',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_105_3_desc},
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

    # Robot 4 Nodes with a unique namespace
    robot_4_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace='/limo_105_4',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_105_4_desc},
        ]
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

    return LaunchDescription([
        robot_3_state_publisher,
        identify_service_3,
        mission_service_3,
        robot_4_state_publisher,
        identify_service_4,
        mission_service_4,
    ])
