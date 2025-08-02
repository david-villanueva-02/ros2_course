#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import CountUntil


class CountUntilClient(Node): 
    def __init__(self):
        super().__init__("count_until_client") 

        # Declare the node as action client
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")

    def send_goal(self, target_number, period):
        # Wait for the server
        self.count_until_client_.wait_for_server()

        # Create the goal 
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # Send the goal asynchronously
        self.get_logger().info("Sending goal...")
        self.count_until_client_. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback). \
                add_done_callback(self.goal_response_callback)
        
        # Send a cancel request after 2 seconds
        # self.timer_ = self.create_timer(2.0, self.cancel_goal)
    
    def cancel_goal(self):
        self.get_logger().info("Sending a cancel request")
        self.goal_handle_.cancel_goal_async()
        self.timer_.cancel() # To only run once

    def goal_response_callback(self, future):
        # Async callback for the goal acceptance
        # Check whether the goal is accepted 
        self.goal_handle_: ClientGoalHandle= future.result()

        # If the goal is accepted
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else: # Goal rejected
            self.get_logger().warn("Goal got rejected")


    def goal_result_callback(self, future):
        # Async callback for the result of the goal

        status = future.result().status
        result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")

        self.get_logger().info(f"Result: {result.reached_number}")

    def goal_feedback_callback(self, feedback_msg):
        '''Callback method to receive feedback in action execution'''
        number = feedback_msg.feedback.current_number
        self.get_logger().info(f"Got feedback {number}")

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClient()

    # Action server call from client
    node.send_goal(10, 1.0)
    # node.send_goal(15, 1.0)
    # node.send_goal(4, 1.0)
    # node.send_goal(5, 1.0)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()