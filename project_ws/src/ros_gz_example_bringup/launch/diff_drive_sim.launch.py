#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Arguments to choose the drive mode for each robot
    drive_mode_3 = LaunchConfiguration('drive_mode_3', default='diff_drive')
    drive_mode_4 = LaunchConfiguration('drive_mode_4', default='diff_drive')

    pkg_robot_3 = LaunchConfiguration('pkg_robot_3', default='limo_105_3_' + str(drive_mode_3))
    sdf_file_3 = os.path.join(get_package_share_directory(pkg_robot_3), 'model.sdf') 
    with open(sdf_file_3, 'r') as infp:
        robot_105_3_desc = infp.read()

    # Load the appropriate SDF file for Robot 4 based on the drive mode
    pkg_robot_4 = LaunchConfiguration('pkg_robot_4', default='limo_105_4_' + str(drive_mode_4))
    sdf_file_4 = os.path.join(get_package_share_directory(pkg_robot_4), 'model.sdf')
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
