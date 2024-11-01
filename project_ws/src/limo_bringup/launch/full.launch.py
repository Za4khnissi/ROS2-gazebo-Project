from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    limo_bringup_dir = get_package_share_directory('limo_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    explore_lite_dir = get_package_share_directory('explore_lite')

    slam_param_file_name = 'slam.yaml'
    slam_param_path = os.path.join(
            limo_bringup_dir,
            'param',
            slam_param_file_name)
    # Define launch files
    limo_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(limo_bringup_dir, 'launch', 'limo_start.launch.py')
        ])
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={'slam_params_file': slam_param_path}.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(limo_bringup_dir, 'launch', 'navigation2.launch.py')
        ])
    )

    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(explore_lite_dir, 'launch', 'explore.launch.py')
        ])
    )

    # Create the launch description with timed actions
    ld = LaunchDescription()

    # Add limo_start
    ld.add_action(limo_start)

    # Add slam_toolbox with 5 second delay
    ld.add_action(TimerAction(
        period=7.0,
        actions=[slam_toolbox]
    ))

    # Add navigation with 10 second delay
    ld.add_action(TimerAction(
        period=12.0,
        actions=[navigation]
    ))

    # Add explore with 15 second delay
    ld.add_action(TimerAction(
        period=20.0,
        actions=[explore]
    ))

    return ld