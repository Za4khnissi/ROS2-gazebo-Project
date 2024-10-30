from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
                'min_x': -5.0,
                'max_x': 5.0,
                'min_y': -5.0,
                'max_y': 5.0,
                'map_frame': LaunchConfiguration('map_frame'),
                'robot_frame': LaunchConfiguration('robot_frame'),
                'nav_action_server': '/limo_105_3/navigate_to_pose',
                'goal_tolerance': 0.5,
                'min_distance_from_current': 1.0,
                'max_attempts': 100,
                'timeout_delay': 5.0,
                'return_to_init': False,
                'progress_timeout': 30.0,
                'pose_topic': '/limo_105_3/amcl_pose'
            }]
        )
    ])