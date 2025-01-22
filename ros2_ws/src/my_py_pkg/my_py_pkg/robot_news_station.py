#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class RobotNewStation(Node):

    def __init__(self):
        super().__init__('robot_news_station')

        # Parameters
        self.declare_parameter("robot_name", "C3PO")

        self.robot_name_ = self.get_parameter("robot_name").value
        self.publisher_ = self.create_publisher(String, 'robot_news', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Robot News Station has been started')

    def timer_callback(self):
        self.publish_news()

    def publish_news(self):
        msg = String()
        msg.data = 'Hello, this is ' + str(self.robot_name_) + " from Robot News Station"
        self.publisher_.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    node = RobotNewStation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()