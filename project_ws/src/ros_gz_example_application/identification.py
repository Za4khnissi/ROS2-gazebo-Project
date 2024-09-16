#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class IdentificationMovement(Node):
    def __init__(self):
        super().__init__('identification_movement')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_subscriber = self.create_subscription(
            String,
            '/movement_command',
            self.command_callback,
            10
        )
        self.is_identifying = False

    def command_callback(self, msg):
        if msg.data == 'identify':
            self.start_identification()

    def start_identification(self):
        if not self.is_identifying:
            self.is_identifying = True
            self.get_logger().info('Identification movement started.')
            self.rotate_in_place()

    def rotate_in_place(self):
        # Create a Twist message to rotate the robot in place
        rotate_msg = Twist()
        rotate_msg.linear.x = 0.0
        rotate_msg.angular.z = 1.0  # Set this to an appropriate value to rotate the robot
        
        # Send the command for a limited time to achieve a 360-degree rotation
        self.publisher_.publish(rotate_msg)
        self.get_logger().info('Robot is rotating for identification.')

        # Create a timer to stop the rotation after a fixed duration (e.g., 2 seconds)
        self.create_timer(2.0, self.stop_rotation)

    def stop_rotation(self):
        # Stop the robot after rotating
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)
        self.is_identifying = False
        self.get_logger().info('Identification movement stopped.')

def main(args=None):
    rclpy.init(args=args)
    identification_movement = IdentificationMovement()
    rclpy.spin(identification_movement)
    identification_movement.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
