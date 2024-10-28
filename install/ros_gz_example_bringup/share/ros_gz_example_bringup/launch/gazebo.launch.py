#!/usr/bin/env python3
# gazebo.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Setup project paths
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Include Gazebo simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                pkg_project_gazebo,
                'worlds',
                'modified_world.sdf'
            ])
        }.items(),
    )

    return LaunchDescription([
        gz_sim,
    ])
