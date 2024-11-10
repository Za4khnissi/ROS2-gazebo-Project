#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8 as Int8Msg
from ros_gz_example_application.srv import MissionCommand
from enum import IntEnum

PUBLISH_RATE = 1/10  # 10 Hz


class MissionStatus(IntEnum):
    STOP = 0
    STARTED = 1
    RETURNING = 2


class ExploreCommand(IntEnum):
    RESUME = 1
    STOP = 2
    STOP_AND_RETURN = 3


class MissionServiceNode(Node):
    def __init__(self):
        super().__init__('mission_service_node')

        self.srv = self.create_service(MissionCommand, 'mission', self.handle_mission_service)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
        self.explore_resume_pub = self.create_publisher(Int8Msg, 'explore/resume', 10)

        self.mission_status = MissionStatus.STOP

        self.publish_timer = None

        self.get_logger().info('Mission Service Node has been started')

    def handle_mission_service(self, request, response):
        command = request.command

        if command not in [1, 2, 3]:
            response.success = False
            response.message = f'Invalid command: {command}. Valid commands are 1 (start), 2 (stop), 3 (stop and return)'
            return response

        # Command 1: Start mission
        if command == 1:
            if self.mission_status == MissionStatus.STARTED:
                response.success = False
                response.message = 'Mission is already started'
            else:
                self.start_mission()
                response.success = True
                response.message = 'Mission started'

        # Command 2: Stop mission
        elif command == 2:
            if self.mission_status == MissionStatus.STOP:
                response.success = False
                response.message = 'Mission is already stopped'
            else:
                self.stop_mission()
                response.success = True
                response.message = 'Mission stopped'

        # Command 3: Stop and return
        elif command == 3:
            if self.mission_status == MissionStatus.RETURNING:
                response.success = False
                response.message = 'Robot is already returning to start'
            else:
                self.stop_and_return()
                response.success = True
                response.message = 'Mission stopped and returning to start'

        return response

    def start_mission(self):
        self.mission_status = MissionStatus.STARTED
        self.get_logger().info('Starting the exploration')
        self.publish_explore_command(ExploreCommand.RESUME)

    def stop_mission(self):
        self.mission_status = MissionStatus.STOP
        self.get_logger().info('Stopping the exploration')
        self.publish_explore_command(ExploreCommand.STOP)

    def stop_and_return(self):
        self.mission_status = MissionStatus.RETURNING
        self.get_logger().info('Stopping exploration and returning to start')
        self.publish_explore_command(ExploreCommand.STOP_AND_RETURN)

    def publish_explore_command(self, command: ExploreCommand):
        explore_msg = Int8Msg()
        explore_msg.data = command
        self.explore_resume_pub.publish(explore_msg)
        self.get_logger().info(f'Published explore command: {command}')

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.9
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Published random velocity: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()