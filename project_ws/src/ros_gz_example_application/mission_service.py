#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool
from std_msgs.msg import Bool
from enum import Enum

PUBLISH_RATE = 1/10  # 10 Hz


class MissionStatus(Enum):
    STOP = 0
    STARTED = 1


class MissionServiceNode(Node):
    def __init__(self):
        super().__init__('mission_service_node')

        self.srv = self.create_service(SetBool, 'mission', self.handle_mission_service)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
        self.explore_resume_pub = self.create_publisher(Bool, 'explore/resume', 10)

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
        self.get_logger().info('Starting the exploration: publishing True to explore/resume')
        
        # Create message and publish True to start exploration
        explore_msg = Bool()
        explore_msg.data = True
        self.explore_resume_pub.publish(explore_msg)

    def stop(self):
        self.mission_status = MissionStatus.STOP
        self.get_logger().info('Stopping the exploration: publishing False to explore/resume')
        
        # Create message and publish False to stop exploration
        explore_msg = Bool()
        explore_msg.data = False
        self.explore_resume_pub.publish(explore_msg)

    def publish_velocity(self):
        # Create a Twist message with linear velocity x: 0.2
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0

        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Published velocity command: linear.x = 0.2')


def main(args=None):
    rclpy.init(args=args)
    node = MissionServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()