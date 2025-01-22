#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')

        self.counter = 0
        self.call_add_two_ints_server(5, 1)

        self.timer_ = self.create_timer(0.5, self.callback_timer)

    def callback_timer(self):
        self.counter += 1
        self.get_logger().info(str(self.counter))

    def call_add_two_ints_server(self, a, b):
        
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("Waiting for server Add Two Ints")

        # Initial call
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Asryncronus wait
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))
    
    def callback_call_add_two_ints(self, future, a, b):
        try: 
            response = future.result() 
            self.get_logger().info(str(a) + ' + ' + str(b) + ' = ' + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args = None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()