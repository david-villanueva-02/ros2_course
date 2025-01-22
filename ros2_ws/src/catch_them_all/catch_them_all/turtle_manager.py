#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Interfaces
from robot_interfaces.msg import Turtle
from turtlesim.msg import Pose

# Other modules
from math import pi, pow, sqrt

class TurtleManagerNode(Node):

    def __init__(self):
        super().__init__('turtle_manager')

        # Variables
        self.alive_turtles = {}
        self.killer_posision = [0.0, 0.0, 0.0] # Initial position for turtle 1 (killer)
        self.killer_velocity = [0.0, 0.0] # Initial velocity for turtle 1 (killer)
        self.target = Turtle()     # Actual target in a Turtle object

        # Subscribers
        self.new_turtles_subscriber = self.create_subscription(Turtle, 'new_turtle', self.new_turtle_callback, 10)
        self.killed_turtle_subscriber = self.create_subscription(Turtle, 'killed', self.callback_killed_turtle, 10)
        self.killer_posision_subsc = self.create_subscription(Pose, '/turtle1/pose', self.callback_killer_posision, 10)

        # Publishers
        self.target_publisher = self.create_publisher(Turtle, 'target', 10)

    def new_turtle_callback(self, msg):
        '''Stores a new turtle in the dictionary as alive'''
        self.alive_turtles[msg.name] = msg

        # Updates the target as a new turtle is spawned
        new_target: Turtle = self.alive_turtles[self.nearest_turtle()]

        # If the target is the same, it does not publish the message
        if new_target.name == self.target.name: 
            return
        
        # Updates if a new target is found
        self.target = new_target

        # Publishes the new target
        self.target_publisher.publish(self.target)
    
    def nearest_turtle(self):
        '''Returns the name of the nearest turtle'''
        min_distance = float('inf')
        nearest_turtle: str = None
        
        # Iterates through all the turtles and identifies the nearest
        for name, turtle in self.alive_turtles.items():
            distance = sqrt(pow(turtle.x - self.killer_posision[0], 2) + pow(turtle.y - self.killer_posision[1], 2))
            if distance < min_distance:
                min_distance = distance
                nearest_turtle = name

        return nearest_turtle

    def callback_killer_posision(self, msg):
        '''Callback that updates the position of the killer turtle'''
        self.killer_posision = [round(msg.x,2), 
                                round(msg.y,2), 
                                round(msg.theta,2) + pi] # Adjust so the range is from 0 to 2*pi
        self.killer_velocity = [round(msg.linear_velocity, 2), 
                                round(msg.angular_velocity,2)]

        # self.get_logger().info(f"Killer position: x: {msg.x}, y: {msg.y}, theta: {msg.theta}")

    def callback_killed_turtle(self, msg):
        if msg.name not in list(self.alive_turtles.keys()):
            self.get_logger().error("Turtle already dead")
            return
        
        # Removes the turtle
        del self.alive_turtles[msg.name]
        self.get_logger().info(f"Killed turtle: {msg.name}")

        # If there are no alive turtles, it does not publish the message
        if len(self.alive_turtles) == 0: return 

        # Updates the target as turtle was killed
        self.target = self.alive_turtles[self.nearest_turtle()]

        # Publishes the new target
        self.target_publisher.publish(self.target)

def main(args = None):
    rclpy.init(args=args)
    node = TurtleManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()