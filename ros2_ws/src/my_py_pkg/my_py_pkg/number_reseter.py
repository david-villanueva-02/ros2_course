#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.srv import SetBool

class NumberReseter(Node):

    def __init__(self):
        super().__init__('number_reseter')
        self.timer_ = self.create_timer(5, self.callback_timer)

    def callback_timer(self):
        self.call_reset_server(True)

    def call_reset_server(self, flag):
        
        client = self.create_client(SetBool, "reset_counter")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for server Add Two Ints")

        # Initial call
        request = SetBool.Request()
        request.data = flag

        # Asryncronus wait
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_add_two_ints, flag = flag))
    
    def callback_call_add_two_ints(self, future, flag):
        try: 
            response = future.result() 
            self.get_logger().info("Counter reset: " + str(flag))
            self.get_logger().info("Success: " + str(response.success))  
            self.get_logger().info("Message: " + str(response.message))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args = None):
    rclpy.init(args=args)
    node = NumberReseter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()