#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rclpy.client import Client

class LifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")

        time.sleep(5.0)
        node_names_and_namespaces = self.get_node_names_and_namespaces()

        # List containing existing robots
        self.existing_robot_names = []
        self.robot_clients = []

        self.get_logger().info("Currently active nodes:")
        for name, namespace in node_names_and_namespaces:
            full_name = f"{namespace}/{name}" if namespace != '/' else name

            if "robot" in full_name:
                self.get_logger().info(f"Found {full_name} node")
                self.existing_robot_names.append(full_name)
        
        for robot_name in self.existing_robot_names:
            # Create a client for each robot
            service_change_state_name = "/" + robot_name + "/change_state"
            client = self.create_client(ChangeState, service_change_state_name)
            self.robot_clients.append(client)
        
    def change_state(self, client: Client, transition: Transition):
        client.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
    
    def initialization_sequence(self):
        # Unconfigured to Inactive
        for robot_name, robot_client in zip(self.existing_robot_names, self.robot_clients):
            self.get_logger().info(f"Trying to switch to configuring for {robot_name}")
            transition = Transition()
            transition.id = Transition.TRANSITION_CONFIGURE
            transition.label = "configure"
            self.change_state(robot_client, transition)
            self.get_logger().info(f"Configuring OK, now inactive {robot_name}")

        # sleep just for the example
        time.sleep(3)

        # Inactive to Active
        for robot_name, robot_client in zip(self.existing_robot_names, self.robot_clients):
            self.get_logger().info(f"Trying to switch to configuring for {robot_name}")
            transition = Transition()
            transition.id = Transition.TRANSITION_ACTIVATE
            transition.label = "activate"
            self.change_state(robot_client, transition)
            self.get_logger().info(f"Activating OK, now active {robot_name}")


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleNodeManager()
    # rclpy.spin(node)
    node.initialization_sequence()
    rclpy.shutdown()


if __name__ == "__main__":
    main()