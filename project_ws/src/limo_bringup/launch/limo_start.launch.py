import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID')

    ros_namespace = ['limo_105_', EnvironmentVariable('ROBOT_ID')]
    absolute_namespace = f'/{ros_namespace[0]}{ROBOT_ID}'
    namespace=f'{ros_namespace[0]}{ROBOT_ID}'

    map_merge_config = {
        'merging_rate': 10.0,
        'discovery_rate': 2.0,
        'estimation_rate': 10.0,
        'estimation_confidence': 0.6,
        'robot_map_topic': 'map',
        'robot_map_updates_topic': 'map_udpates',
        'robot_namespace': '',
        'merged_map_topic': 'map',
        'world_frame': 'map',
        'known_init_poses': True,
        "/limo_105_1/map_merge/init_pose_x": 0.0,
        "/limo_105_1/map_merge/init_pose_y": 0.0,
        "/limo_105_1/map_merge/init_pose_z": 0.0,
        "/limo_105_1/map_merge/init_pose_yaw": 0.0,

        "/limo_105_2/map_merge/init_pose_x": 1.0,
        "/limo_105_2/map_merge/init_pose_y": 0.0,
        "/limo_105_2/map_merge/init_pose_z": 0.0,
        "/limo_105_2/map_merge/init_pose_yaw": 0.0,
        
    }
    
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='port_name',
                                             default_value='ttyTHS1'),
        launch.actions.DeclareLaunchArgument(name='odom_topic_name',
                                             default_value='odom'),
        launch.actions.DeclareLaunchArgument(name='open_rviz',
                                             default_value='false'),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            on_exit=launch.actions.Shutdown(),
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('open_rviz'))),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            namespace=absolute_namespace,
            arguments=f'0.0 0.0 0.0 0.0 0.0 0.0 {ros_namespace[0]}{ROBOT_ID}/base_link {ros_namespace[0]}{ROBOT_ID}/imu_link'.split(
                ' ')),
        # launch_ros.actions.Node(
        #     package='robot_pose_ekf',
        #     executable='robot_pose_ekf',
        #     name='robot_pose_ekf',
        #     parameters=[
        #         {
        #             'output_frame': 'odom'
        #         },
        #         {
        #             'base_footprint_frame': 'base_link'
        #         }
        #     ]
        # ),
         launch.actions.IncludeLaunchDescription(
             launch.launch_description_sources.PythonLaunchDescriptionSource(
                 os.path.join(get_package_share_directory('limo_base'),
                              'launch/limo_base.launch.py')),
             launch_arguments={
                 'port_name':
                 launch.substitutions.LaunchConfiguration('port_name'),
                 'odom_topic_name':
                 launch.substitutions.LaunchConfiguration('odom_topic_name')
             }.items()),
         launch.actions.IncludeLaunchDescription(
             launch.launch_description_sources.PythonLaunchDescriptionSource(
                 os.path.join(get_package_share_directory('limo_base'),
                              'launch','open_ydlidar_launch.py'))),

        TimerAction(
            period=12.0,
            actions=[
                launch_ros.actions.Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox_1',
                    namespace=absolute_namespace,
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'map_frame': 'map',
                        'odom_frame': f"{namespace}/odom",
                        'base_frame': f"{namespace}/base_link",
                        'scan_topic': f"{absolute_namespace}/scan"
                    }],
                    remappings=[
                        ('/map', f'{absolute_namespace}/map'),
                        ('/map_metadata', f'{absolute_namespace}/map_metadata'),
                        ('/scan', f'{absolute_namespace}/scan'),
                        ('/tf', '/tf'),
                        ('/tf_static', '/tf_static'),
                        ('/odom', f'{absolute_namespace}/odom')
                    ]
                )
            ]
        ),
        TimerAction(
            period=16.0,
            actions=[
                launch_ros.actions.Node(
                    package="multirobot_map_merge",
                    name="map_merge",
                    #namespace=namespace,
                    executable="map_merge",
                    parameters=[
                        map_merge_config,
                        {"use_sim_time": False},
                    ],
                    output="screen",
                    condition=IfCondition(str(ROBOT_ID == "2"))
                )
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()