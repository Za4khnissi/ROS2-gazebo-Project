#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool
from enum import Enum
import random

PUBLISH_RATE = 1/10  # 10 Hz


class MissionStatus(Enum):
    STOP = 0
    STARTED = 1


class MissionServiceNode(Node):
    def __init__(self):
        super().__init__('mission_service_node')

        self.srv = self.create_service(SetBool, 'mission', self.handle_mission_service)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))

        self.mission_status = MissionStatus.STOP

        self.publish_timer = None

        self.get_logger().info('Mission Service Node has been started')

    def handle_mission_service(self, request, response):
        if request.data and self.mission_status != MissionStatus.STARTED:
            self.start()
            response.success = True
            response.message = 'Mission started'
        elif not request.data and self.mission_status != MissionStatus.STOP:
            self.stop()
            response.success = True
            response.message = 'Mission stopped'
        else:
            response.success = False
            response.message = 'No change in mission status'

        return response

    def start(self):
        self.mission_status = MissionStatus.STARTED
        self.get_logger().info('Starting the mission: publishing random velocities to /cmd_vel')

        # Create a timer to publish velocity at the defined rate
        self.publish_timer = self.create_timer(PUBLISH_RATE, self.publish_velocity)

    def stop(self):
        self.mission_status = MissionStatus.STOP
        self.get_logger().info('Stopping the mission: stopping publishing to /cmd_vel')

        # Cancel the timer to stop publishing velocity
        if self.publish_timer is not None:
            self.publish_timer.cancel()
            self.publish_timer = None
            self.cmd_vel_pub.publish(Twist())  # Publish zero velocity to stop the robot

    def publish_velocity(self):
        msg = Twist()

        msg.linear.x = random.uniform(0.1, 0.5)

        msg.angular.z = random.uniform(-0.5, 0.5)

        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Published random velocity: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
