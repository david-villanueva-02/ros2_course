#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import SetLed
from time import time
from functools import partial

class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')

        # Variables
        self.battery_level = 0

        # Timer
        self.main_timer = self.create_timer(10, self.timer_callback)

        #test 2
        self.test = 2

    def timer_callback(self):
        self.get_logger().info("Tic-tac")
        initial = time()
        while time() - initial < 4: pass # Battery empty

        self.call_set_led_server(True)

        while time() - initial < 10: pass # Battery full again
        
        self.call_set_led_server(False)

    def call_set_led_server(self, state):
        
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(timeout_sec=1):
            self.get_logger().warn("Waiting for server Set Led")

        # Initial call
        request = SetLed.Request()
        request.state = state

        # Asryncronus wait
        future = client.call_async(request)
        # future.add_done_callback(lambda: self.callback_call_led_server(state=state))
        future.add_done_callback(partial(self.callback_call_led_server, state=state))
    
    def callback_call_led_server(self, future, state):
        try: 
            response = future.result() 
            self.get_logger().info('Resquest successful: ' + str(response.success))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args = None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()