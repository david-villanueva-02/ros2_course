#!/usr/bin/env python3
import rclpy
# from rclpy.node import Node # Conventional node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from example_interfaces.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("In constructor")
        self.number_ = 1
        self.publish_frequency_ = 1.0

        # Declare ROS2 parameters
        self.number_publisher_ = None
        self.number_timer_ = None

    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        '''
        Transition from unconfigured to inactive.
        Initialize ROS2 communication and connect to hardware
        '''
        self.get_logger().info("In on_configure")
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer_ = self.create_timer(1.0 / self.publish_frequency_, self.publish_number)
        self.number_timer_.cancel()

        # return TransitionCallbackReturn.ERROR # Triggers an error for testing
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, previous_state: LifecycleState):
        '''
        Transition from inactive to unconfigured
        Destroys ROS2 communication and disconnect from hardware
        '''
        self.get_logger().info("In on_cleanup")
        self.destroy_lifecycle_publisher(self.number_publisher_) # Destroys the publisher
        self.destroy_timer(self.number_timer_) # Destroys the timer

        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, previous_state:LifecycleState):
        '''
        Transition from inactive to active. 
        Activate/enable hardware and ROS2 communication
        '''
        self.number_timer_.reset()
        self.get_logger().info("In on_activate")

        # Calls the original method with extra functionalities instead of overwriting it
        return super().on_activate(previous_state)

    def on_deactivate(self, previous_state):
        '''
        Transition from active to inactive
        Disable hardware and stop ROS2 communication. Opposite of on_activate
        '''
        self.number_timer_.cancel()
        self.get_logger().info("In on_deactivate")

        # Calls the original method with extra functionalities instead of overwriting it
        return super().on_deactivate(previous_state)
    
    def on_shutdown(self, previous_state):
        '''
        Transition from any state to shutdown
        '''
        self.get_logger().info("In on_shutdown")
        self.destroy_lifecycle_publisher(self.number_publisher_) # Destroys the publisher
        self.destroy_timer(self.number_timer_) # Destroys the timer
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, previous_state):
        '''
        Callback triggered when an error ocurrs during any transition
        Process errors and return SUCCESS OR FAILURE depending on the errors
        '''
        self.get_logger().info("in on_error")
        self.destroy_lifecycle_publisher(self.number_publisher_) # Destroys the publisher
        self.destroy_timer(self.number_timer_) # Destroys the timer

        # Do some checks (communication, hardware, logic, etc)
        # If the error is fixed, return SUCCESS, otherwise return FAILURE

        return TransitionCallbackReturn.SUCCESS

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
