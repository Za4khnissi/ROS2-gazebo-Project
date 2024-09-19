import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define namespace based on the environment variable
    # get env variable ROBOT_ID
    ROBOT_ID = os.getenv('ROBOT_ID')

    ros_namespace = ['limo_105_', EnvironmentVariable('ROBOT_ID')]

    namespace_string = f'/{ros_namespace[0]}/{ROBOT_ID}'

    port_name_arg = DeclareLaunchArgument('port_name', default_value='ttyUSB1',
                                          description='usb bus name, e.g. ttyUSB1')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
                                           description='Odometry frame id')
    base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
                                                description='Base link frame id')
    odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
                                           description='Odometry topic name')
    # sim_control_rate_arg = DeclareLaunchArgument('control_rate', default_value='50',
    #                                             description='Simulation control loop update rate')
    # pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
    #                                              description='Odometry TF')
    
    port_name_arg = LaunchConfiguration('port_name', default='ttyTHS1')
    odom_frame_arg = LaunchConfiguration('odom_frame', default='odom')
    base_link_frame_arg = LaunchConfiguration('base_frame', default='base_link')
    pub_odom_tf_arg_ = LaunchConfiguration('pub_odom_tf', default='true')

    remapping = [
        ('odom', '/wheel/odom'),
        ('/cmd_vel', namespace_string),
    ]

    # Define limo_base_node with the namespace applied
    limo_base_node = Node(
        package='limo_base',
        executable='limo_base',  # foxy executable='limo_base',
        output='screen',
        name='limo_base_node',
        namespace=ros_namespace,
        emulate_tty=True,
        parameters=[{
            # 'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
            'port_name': port_name_arg,
            'odom_frame': odom_frame_arg,
            'base_frame': base_link_frame_arg,
            'pub_odom_tf': pub_odom_tf_arg_,
            'use_mcnamu': False
        }],
        remappings=remapping
    )

    # Define identify_service node with the namespace applied
    identify_service = Node(
        package='ros_gz_example_application',
        executable='identify_service.py',
        name='identify_service',
        namespace=ros_namespace,
        output='screen',
    )

    # Define mission_service node with the namespace applied
    mission_service = Node(
        package='ros_gz_example_application',
        executable='mission_service.py',
        name='mission_service',
        namespace=ros_namespace,
        output='screen',
    )

    return LaunchDescription([
        # DeclareLaunchArgument('pub_odom_tf',default_value=pub_odom_tf_arg_,description='TF'),
        limo_base_node,
        identify_service,
        mission_service
    ])
