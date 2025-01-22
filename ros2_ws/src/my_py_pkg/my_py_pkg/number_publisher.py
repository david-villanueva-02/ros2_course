#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64

class NumberPublisher(Node):

    def __init__(self):
        super().__init__('number_publisher')
        self.declare_parameter("number", 2)
        self.declare_parameter("period", 1.0)

        # Variables
        self.number_ = self.get_parameter("number").value
        self.period = self.get_parameter("period").value

        self.publisher_ = self.create_publisher(Int64, 'number', 10)
        self.timer_ = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info('Number Publisher has been started')

    def timer_callback(self):
        self.publish_number()
    
    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()