#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import HardwareStatus
from example_interfaces.msg import String

class HardwareStatausPublisher(Node):

    def __init__(self):
        super().__init__('hardware_status_publisher')
        self.hw_status_publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer_ = self.create_timer(1, self.callback_timer)

        self.get_logger().info('Hardware Status Publisher has been started')
    
    def callback_timer(self):
        self.publish_hardware_status()

    def publish_hardware_status(self):
        msg = HardwareStatus()
        msg.temperature = 45
        msg.are_motors_ready = True
        msg.debug_message = 'Debug message'

        self.hw_status_publisher_.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    node = HardwareStatausPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()