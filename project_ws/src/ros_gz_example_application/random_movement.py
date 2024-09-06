#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import random

class RandomMovement(Node):
    def __init__(self):
        super().__init__('randommovement')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_subscriber = self.create_subscription(
            String,
            '/movement_command',
            self.command_callback,
            10
        )
        self.timer = None
        self.is_moving = False
    def command_callback(self, msg):
        if msg.data == 'start':
            self.start_movement()
        elif msg.data == 'stop':
            self.stop_movement()

    def start_movement(self):
        if not self.is_moving:
            self.timer = self.create_timer(0.1, self.move_randomly)
            self.is_moving = True
            self.get_logger().info('Random movement started.')

    def stop_movement(self):
        if self.is_moving:
            self.timer.cancel()
            self.is_moving = False
            self.get_logger().info('Random movement stopped.')
            self.publish_stop_command()

    def move_randomly(self):
        msg = Twist()
        msg.linear.x = random.uniform(0, 0.5)
        msg.angular.z = random.uniform(-1.0, 1.0)
        self.publisher_.publish(msg)

    def publish_stop_command(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    random_movement = RandomMovement()
    rclpy.spin(random_movement)
    random_movement.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
