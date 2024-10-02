#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String

class DriveModeService(Node):
    def __init__(self):
        super().__init__('drive_mode_service')

        self.srv = self.create_service(SetBool, 'drive_mode', self.change_drive_mode_callback)

        self.drive_mode_pub = self.create_publisher(String, 'drive_mode', 10)

        self.current_drive_mode = 'diff_drive'
        self.get_logger().info(f'Started with drive mode: {self.current_drive_mode}')

    def change_drive_mode_callback(self, request, response):

        if request.data:
            new_mode = 'ackermann'
        else:
            new_mode = 'diff_drive'

        if new_mode != self.current_drive_mode:
            self.get_logger().info(f'Switching drive mode from {self.current_drive_mode} to {new_mode}')
            self.current_drive_mode = new_mode
            msg = String()
            msg.data = new_mode
            self.drive_mode_pub.publish(msg)
            response.success = True
            response.message = f'Drive mode switched to {new_mode}'
        else:
            response.success = False
            response.message = f'Already in {new_mode} mode'
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DriveModeService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
