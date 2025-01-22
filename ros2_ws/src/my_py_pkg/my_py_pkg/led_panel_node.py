#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import SetLed
from robot_interfaces.msg import LedState

class LedPannelNode(Node):

    def __init__(self):
        super().__init__('led_pannel_node')

        # Parameters
        self.declare_parameter("led_states", [0, 0, 0])

        # Variables
        self.led_pannel = self.get_parameter("led_states").value

        # Servers
        self.set_led_server = self.create_service(SetLed, 'set_led', self.callback_set_led)
        self.get_logger().info("Led Pannel Node has been started")

        # Publishers
        self.led_pannel_publisher_ = self.create_publisher(LedState, 'led_pannel_state', 10)

    def publish_led_state(self):
        msg = LedState()
        msg.led_state = self.led_pannel
        self.led_pannel_publisher_.publish(msg)

    def callback_set_led(self, request, response):
        try: 
            if request.state:
                self.led_pannel[2] = 1
            else: 
                self.led_pannel[2] = 0

            self.publish_led_state()

            response.success = True
        
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        
        return response

def main(args = None):
    rclpy.init(args=args)
    node = LedPannelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()