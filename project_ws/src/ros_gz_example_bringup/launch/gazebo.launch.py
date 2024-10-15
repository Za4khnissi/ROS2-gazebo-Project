#!/usr/bin/env python3
# gazebo.launch.py

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression

def generate_launch_description():
    # Declare launch arguments without default values
    drive_mode_3_arg = DeclareLaunchArgument('drive_mode_3', description='Drive mode for Robot 3')
    drive_mode_4_arg = DeclareLaunchArgument('drive_mode_4', description='Drive mode for Robot 4')

    # Get the drive modes as LaunchConfigurations
    drive_mode_3 = LaunchConfiguration('drive_mode_3')
    drive_mode_4 = LaunchConfiguration('drive_mode_4')

    # Setup project paths
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Logic to select the world file based on drive modes
    world_file_name = PythonExpression([
        "'ackermann_diff.sdf' if '", drive_mode_3, "' == 'ackermann' and '", drive_mode_4, "' == 'diff_drive' else ",
        "'ackermann.sdf' if '", drive_mode_3, "' == 'ackermann' and '", drive_mode_4, "' == 'ackermann' else ",
        "'diff_ackermann.sdf' if '", drive_mode_3, "' == 'diff_drive' and '", drive_mode_4, "' == 'ackermann' else ",
        "'diff_drive.sdf'"
    ])

    # Path to the world file
    world_file = PathJoinSubstitution([
        pkg_project_gazebo,
        'worlds',
        world_file_name
    ])

    # Include Gazebo simulation with the selected world file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': world_file
        }.items(),
    )

    # Log the selected world file
    log_info = LogInfo(msg=['Launching Gazebo with world file: ', world_file])

    # Return the launch description
    return LaunchDescription([
        drive_mode_3_arg,
        drive_mode_4_arg,
        log_info,
        gz_sim,
    ])
