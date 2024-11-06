#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup_robot_descriptions(context):
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

    # Define robot nodes using the loaded descriptions
    robot_3_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace='limo_105_3',
        parameters=[{'use_sim_time': True, 'robot_description': robot_105_3_desc}]
    )

    robot_4_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace='limo_105_4',
        parameters=[{'use_sim_time': True, 'robot_description': robot_105_4_desc}]
    )

    # Define Identify and Mission services for Robots 3 and 4
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

    drive_mode_service_3 = Node(
        package='ros_gz_example_application',
        executable='drive_mode_service.py',
        name='drive_mode_service',
        output='screen',
        namespace='limo_105_3',
        parameters=[{'initial_drive_mode': drive_mode_3}]
    )

    drive_mode_service_4 = Node(
        package='ros_gz_example_application',
        executable='drive_mode_service.py',
        name='drive_mode_service',
        output='screen',
        namespace='limo_105_4',
        parameters=[{'initial_drive_mode': drive_mode_4}]
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

    return [
        robot_3_state_publisher,
        robot_4_state_publisher,
        identify_service_3,
        mission_service_3,
        identify_service_4,
        mission_service_4,
        drive_mode_service_3,
        drive_mode_service_4,
        battery_node_3,
        battery_node_4
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
