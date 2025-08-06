#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # Robot parameters for differential drive
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel_radius %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel_separation %f" % self.wheel_separation_)

        self.wheel_cmd_publisher = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.velocity_subscriber = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.velocity_callback, 10)

        # Matrix
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("The conversion matrix is %s" %self.speed_conversion_)
    
    def velocity_callback(self, msg: TwistStamped):
        '''
        Callback for velocity commands
        Calculates the wheel velocities as a function of the velocity commands given to the robot.
        Inverse Kinematics
        '''
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        
        wheel_speed = np.multiply(np.linalg.inv(self.speed_conversion_), robot_speed)
        
        # Set message to send to the wheels 
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        # Publish message
        self.wheel_cmd_publisher.publish(wheel_speed_msg)

def main():
    rclpy.init()
    simple_subscriber = SimpleController()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown() 

if __name__ == "__main__":
    main()