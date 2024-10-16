#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess

class DriveModeService(Node):
    def __init__(self):
        super().__init__('drive_mode_service')

        # Declare and get the initial drive mode parameters for Robot 3 and Robot 4
        self.declare_parameter('initial_drive_mode', 'diff_drive')
        self.robot_3_mode = self.get_parameter('initial_drive_mode').get_parameter_value().string_value
        self.robot_4_mode = self.get_parameter('initial_drive_mode').get_parameter_value().string_value

        # Create services for changing the drive modes
        self.srv_robot_3 = self.create_service(SetBool, 'robot_3_drive_mode', self.change_drive_mode_robot_3)
        self.srv_robot_4 = self.create_service(SetBool, 'robot_4_drive_mode', self.change_drive_mode_robot_4)

    def change_drive_mode_robot_3(self, request, response):
        new_mode = 'ackermann' if request.data else 'diff_drive'
        if self.robot_3_mode != new_mode:
            self.robot_3_mode = new_mode
            self.restart_gazebo_with_current_modes()
            response.success = True
            response.message = f"Switched Robot 3 to {self.robot_3_mode} mode."
            self.get_logger().info(response.message)
        else:
            response.success = True
            response.message = f"Robot 3 is already in {self.robot_3_mode} mode. No change needed."
        return response

    def change_drive_mode_robot_4(self, request, response):
        new_mode = 'ackermann' if request.data else 'diff_drive'
        if self.robot_4_mode != new_mode:
            self.robot_4_mode = new_mode
            self.restart_gazebo_with_current_modes()
            response.success = True
            response.message = f"Switched Robot 4 to {self.robot_4_mode} mode."
            self.get_logger().info(response.message)
        else:
            response.success = True
            response.message = f"Robot 4 is already in {self.robot_4_mode} mode. No change needed."
        return response

    def restart_gazebo_with_current_modes(self):
        # Launch the Gazebo simulation with the updated drive modes for both robots
        launch_script_path = os.path.expanduser("~/inf3995/launch_gazebo.sh")
        command = [launch_script_path, self.robot_3_mode, self.robot_4_mode]
        
        try:
            subprocess.run(command, check=True)
            self.get_logger().info(f"Gazebo relaunched with Robot 3 mode: {self.robot_3_mode} and Robot 4 mode: {self.robot_4_mode}")
            return True
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to relaunch Gazebo with new settings: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    service_node = DriveModeService()
    rclpy.spin(service_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
