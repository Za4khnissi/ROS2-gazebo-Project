#!/usr/bin/env python3
# diff_drive_sim.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Setup project paths
    pkg_project_description = get_package_share_directory('ros_gz_example_description')

    # Load the SDF file from the "description" package
    sdf_file = os.path.join(pkg_project_description, 'models', 'limo_105_3', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_105_3_desc = infp.read()

    # Load the SDF file from the "description" package
    sdf_file = os.path.join(pkg_project_description, 'models', 'limo_105_4', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_105_4_desc = infp.read()

    robot_id = LaunchConfiguration('ROBOT_ID')
    ros_namespace = [TextSubstitution(text='limo_105_'), robot_id]

    # Robot 3 State Publisher
    robot_3_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace=ros_namespace,
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_105_3_desc},
        ]
    )

    # Robot 4 State Publisher
    robot_4_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace=ros_namespace,
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_105_4_desc},
        ]
    )

   # Identify and Mission Services for Robot 3
    identify_service_3 = Node(
        package='ros_gz_example_application',
        executable='identify_service.py',
        name='identify_service',
        output='screen',
        namespace=ros_namespace
    )

    mission_service_3 = Node(
        package='ros_gz_example_application',
        executable='mission_service.py',
        name='mission_service',
        output='screen',
        namespace=ros_namespace
    )

    # Identify and Mission Services for Robot 4
    identify_service_4 = Node(
        package='ros_gz_example_application',
        executable='identify_service.py',
        name='identify_service',
        output='screen',
        namespace=ros_namespace
    )

    mission_service_4 = Node(
        package='ros_gz_example_application',
        executable='mission_service.py',
        name='mission_service',
        output='screen',
        namespace=ros_namespace
    )

    return LaunchDescription([
        robot_3_state_publisher,
        robot_4_state_publisher,
        identify_service_3,
        mission_service_3,
        identify_service_4,
        mission_service_4,
    ])
