#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import os

class IdentifyService(Node):
    def __init__(self):
        super().__init__('identify_service')
        self.srv = self.create_service(Trigger, 'identify', self.identify_callback)

        self.sound_file = os.path.join(os.path.dirname(__file__), 'sound.wav')

        if not os.path.exists(self.sound_file):
            self.get_logger().error(f"Sound file not found: {self.sound_file}")
        else:
            self.get_logger().info(f"Loaded sound file: {self.sound_file}")

    def identify_callback(self, request, response):
        self.get_logger().info('Identify service called. Playing sound...')

        if os.path.exists(self.sound_file):
            command = f'mpg123 "{self.sound_file}"'
            os.system(command)

            response.success = True
            response.message = "Robot identification successful! Sound played."
        else:
            response.success = False
            response.message = "Sound file missing. Cannot play identification sound."

        return response

def main(args=None):
    rclpy.init(args=args)
    node = IdentifyService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
