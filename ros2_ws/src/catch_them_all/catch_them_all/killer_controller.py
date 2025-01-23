#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Test

# Interfaces
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from robot_interfaces.msg import Turtle

# Other modules
from math import pow, sqrt, atan2, pi
from functools import partial

class KillerControllerNode(Node):

    def __init__(self):
        super().__init__('killer_controller')

        # Parameters
        self.declare_parameter("sample_time", 0.01) # 100 Hz by default
        self.declare_parameter("k_gain_x", 2.0)
        self.declare_parameter("k_gain_theta", 6.0)
        self.declare_parameter("killing_distance", 0.5)
        self.declare_parameter("abs_gain", 8.5)
        # self.declare_parameter("debug_flag", False)

        # Variables
        self.sample_time = self.get_parameter("sample_time").value  
        self.k_gain_x = self.get_parameter("k_gain_x").value
        self.k_gain_theta = self.get_parameter("k_gain_theta").value
        self.killing_distance = self.get_parameter("killing_distance").value
        self.abs_gain = self.get_parameter("abs_gain").value
        self.target = None
        self.killer_posision = [0.0, 0.0, 0.0] # Initial position for turtle 1 (killer)
        self.killer_velocity = [0.0, 0.0] # Initial velocity for turtle 1 (killer)

        # Publishers
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.killer_publisher = self.create_publisher(Turtle, 'killed', 10)

        # Subscribers
        self.killer_posision_subsc = self.create_subscription(Pose, '/turtle1/pose', self.callback_killer_posision, 10)
        self.target_subsc = self.create_subscription(Turtle, 'target', self.callback_target, 10)

        # Timers
        self.timer_controller = self.create_timer(self.sample_time, self.move_turtle)
    
    def callback_killer_posision(self, msg):
        '''Callback that updates the position of the killer turtle'''
        self.killer_posision = [round(msg.x,2), 
                                round(msg.y,2), 
                                round(msg.theta,2)] # Adjust so the range is from 0 to 2*pi
        self.killer_velocity = [round(msg.linear_velocity, 2), 
                                round(msg.angular_velocity,2)]

        # self.get_logger().info(f"Killer position: x: {msg.x}, y: {msg.y}, theta: {msg.theta}")
    
    def callback_target(self, msg):
        '''Callback that updates the target of the killer turtle'''
        self.target = msg

    def move_turtle(self):
        '''Method that moves the turtle one step based on a P controller'''
        if self.target == None: return

        # Calculates input data for closed loop control in angular and linear velocity
        dist_x = self.target.x - self.killer_posision[0]
        dist_y = self.target.y - self.killer_posision[1]
        distance = sqrt(dist_x * dist_x + dist_y * dist_y)

        # Sets the message
        msg = Twist()

        if distance > self.killing_distance:
            # position
            msg.linear.x = self.k_gain_x*distance

            # orientation
            goal_theta = atan2(dist_y, dist_x)
            diff = goal_theta - self.killer_posision[2]
            if diff > pi:
                diff -= 2*pi
            elif diff < -pi:
                diff += 2*pi

            msg.angular.z = self.k_gain_theta*diff
        else:
            # target reached!
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_kill_service(self.target.name)
            self.killer_publisher.publish(self.target)
            self.target = None

        # Publishes the message and moves the turtle
        self.velocity_publisher.publish(msg)

    def call_kill_service(self, name):
        '''Calls the spawn service'''
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for server Kill")

        # Initial call
        request = Kill.Request()
        request.name = name

        # Asryncronus wait
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_calll_kill_service, name = name))
    
    def callback_calll_kill_service(self, future, name):
        '''Waits for the response asyncronously'''
        try: 
            response = future.result() # Should be empty
            self.get_logger().info('Killed ' + name)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args = None):
    rclpy.init(args=args)
    node = KillerControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()