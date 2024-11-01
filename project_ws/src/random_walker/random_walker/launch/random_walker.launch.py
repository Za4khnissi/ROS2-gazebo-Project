import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID')

    ros_namespace = ['limo_105_', EnvironmentVariable('ROBOT_ID')]
    absolute_namespace = f'/{ros_namespace[0]}{ROBOT_ID}'
    namespace = f'{ros_namespace[0]}{ROBOT_ID}'

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame id'
        ),
        DeclareLaunchArgument(
            'robot_frame',
            default_value='limo_105_3',
            description='Robot base frame id'
        ),

        Node(
            package='random_walker',
            executable='random_walker_node',
            name='random_walker',
            output='screen',
            parameters=[{
                'min_x': -100.0,
                'max_x': 100.0,
                'min_y': -100.0,
                'max_y': 100.0,
                'map_frame': "map",
                'robot_frame': f"{namespace}/base_link",
                'nav_action_server': f"/{namespace}/navigate_to_pose",
                'goal_tolerance': 0.5,
                'min_distance_from_current': 0.3,
                'max_attempts': 3,
                'timeout_delay': 100.0,
                'return_to_init': False,
                'progress_timeout': 30.0,
                'pose_topic': f"/{namespace}/amcl_pose"
            }]
        )
    ])