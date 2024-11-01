#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import LogInfo
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


import lifecycle_msgs.msg
import os


def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID')

    ros_namespace = ['limo_105_', EnvironmentVariable('ROBOT_ID')]
    absolute_namespace = f'/{ros_namespace[0]}{ROBOT_ID}'
    namespace = f'{ros_namespace[0]}{ROBOT_ID}'

    share_dir = get_package_share_directory('limo_bringup')
    #parameter_file = LaunchConfiguration('params_file')
    #node_name = 'ydlidar_ros2_driver_node'

    params_file = os.path.join(share_dir, 'param', 'ydlidar.yaml')
    # params_declare = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(share_dir, 'param', 'ydlidar.yaml'),
    #     description='FPath to the ROS2 parameters file to use.')
    
    # params_file = ReplaceString(
    #     source_file=params_file,
    #     replacements={'<robot_namespace>': (namespace)},
    #     )

    # configured_params = ParameterFile(
    #     RewrittenYaml(
    #         source_file=params_file,
    #         root_key=ros_namespace,
    #         param_rewrites={},
    #         convert_types=True),
    #     allow_substs=True)


    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        namespace=absolute_namespace,
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': '/dev/ttyUSB0',
            'frame_id': f'{namespace}/laser_frame',
            'ignore_array': '',
            'baudrate': 115200,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 3,
            'abnormal_check_count': 4,
            'fixed_resolution': True,
            'reversion': True,
            'inverted': True,
            'auto_reconnect': True,
            'isSingleChannel': True,
            'intensity': False,
            'support_motor_dtr': True,
            'angle_max': 180.0,
            'angle_min': -180.0,
            'range_max': 12.0,
            'range_min': 0.1,
            'frequency': 10.0,
            'invalid_range_is_inf': False
        }]
    )
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=absolute_namespace,
        name='static_tf_pub_laser',
        arguments=[
            '0', '0', '0.02', '0', '0', '0', '1', f'{ros_namespace[0]}{ROBOT_ID}/base_link', f'{ros_namespace[0]}{ROBOT_ID}/laser_frame'
        ],
    )

    return LaunchDescription([
        #params_declare,
        driver_node,
        tf2_node,
    ])