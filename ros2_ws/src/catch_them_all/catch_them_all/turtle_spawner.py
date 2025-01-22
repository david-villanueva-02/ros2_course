#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Interfaces
from turtlesim.srv import Spawn
from robot_interfaces.msg import Turtle

# Other modules
from random import uniform
from functools import partial

class TurtleSpawnerNode(Node):

    def __init__(self):
        super().__init__('turtle_spawner')

        # Parameters
        self.declare_parameter("spawning_period", 1.0)

        # Local variables
        self.spawning_period = self.get_parameter("spawning_period").value
        self.turtles_alive = {} # Empty dictionary
        self.counter = 2        # turtle1 is default turtle

        # Publishers
        self.new_turtle_publisher = self.create_publisher(Turtle, 'new_turtle', 10)

        # Timer to spawn new turtles
        self.spawn_timer = self.create_timer(self.spawning_period, self.spawn_turtle)

    def spawn_turtle(self):
        '''Spawns a new turtle with a random location and controlled name'''
        # Generates name, location and orientation
        name = "turtle" + str(self.counter)
        x, y = uniform(0.5, 10.5), uniform(0.5, 10.5)
        theta = uniform(0, 6.28)

        # Spawns the turtle
        self.call_spawn_service(x, y, theta, name)

        # Publishes the new turtle
        msg = Turtle()
        msg.name = name
        msg.x = x
        msg.y = y
        self.new_turtle_publisher.publish(msg)

        # Updates counter
        self.counter += 1

    def call_spawn_service(self, x, y, theta, name):
        '''Calls the spawn service'''
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for server Add Two Ints")

        # Initial call
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.name = name
        request.theta = theta

        # Asryncronus wait
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_calll_spawn_service, x = x, y = y, theta = theta, name = name))
    
    def callback_calll_spawn_service(self, future, x, y, theta, name):
        '''Waits for the response asyncronously'''
        try: 
            response = future.result() 
            self.get_logger().info('Spawned ' + response.name)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args = None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()