#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from limo_msgs.msg import LimoStatus  
from std_msgs.msg import Float32

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        
        self.declare_parameter('is_simulation', True)
        self.is_simulation = self.get_parameter('is_simulation').get_parameter_value().bool_value

        if self.is_simulation:
            self.get_logger().info('simulation.')
            self.battery_publisher = self.create_publisher(Float32, 'battery_level', 10)
            self.battery_level = 100.0
            self.battery_decrease_rate = 10.0
            self.timer = self.create_timer(60.0, self.update_battery_level)
        else:
            self.get_logger().info('Robot physique.')
            self.subscription = self.create_subscription(
                LimoStatus,
                '/limo_status',
                self.physical_battery_callback,
                10
            )
            self.battery_publisher = self.create_publisher(Float32, 'battery_level', 10)

    def update_battery_level(self):
        if self.battery_level > 0:
            self.battery_level -= self.battery_decrease_rate
            if self.battery_level < 0:
                self.battery_level = 0
        
        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_publisher.publish(battery_msg)
        self.get_logger().info(f'Simulated Battery level: {self.battery_level}%')

    def physical_battery_callback(self, msg):
        voltage = msg.battery_voltage
        battery_percentage = self.voltage_to_percentage(voltage)
        #self.get_logger().info(f'Physical Battery Voltage: {voltage}V, Estimated Percentage: {battery_percentage}%')
        
        battery_msg = Float32()
        battery_msg.data = battery_percentage
        self.battery_publisher.publish(battery_msg)

    def voltage_to_percentage(self, voltage):
        full_charge_voltage = 12.6
        empty_charge_voltage = 8.25
        if voltage >= full_charge_voltage:
            return 100
        elif voltage <= empty_charge_voltage:
            return 0
        else:
            return ((voltage - empty_charge_voltage) / (full_charge_voltage - empty_charge_voltage)) * 100

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
