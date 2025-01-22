#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class MyCustomNode(Node):

    def __init__(self):
        super().__init__('number_counter')

        self.counter = 0
        self.number_subscriber = self.create_subscription(Int64, "number", self.callback_number, 10)
        self.publisher_ = self.create_publisher(Int64, 'number_counter', 10)

        # Service to reset the counter
        self.server_ = self.create_service(SetBool, 'reset_counter', self.callback_reset_counter)

    def callback_number(self, msg):
        self.counter += msg.data
        self.get_logger().info(f'Counter: {self.counter}')
        self.publish_counter()

    def publish_counter(self):
        msg = Int64()
        msg.data = self.counter
        self.publisher_.publish(msg)

    def callback_reset_counter(self, request, response):
        if request.data:
            self.counter = 0
            response.success = True
            response.message = 'Counter has been reset'
        else:
            response.success = False
            response.message = 'No reset'
        return response

def main(args = None):
    rclpy.init(args=args)
    node = MyCustomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()