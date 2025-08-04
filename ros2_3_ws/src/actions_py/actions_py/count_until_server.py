#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from time import sleep
import threading


class CountUntilServer(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()

        self.goal_queue_ = []

        # Action server creation
        self.count_until_server_ = ActionServer(self, 
                                                CountUntil, 
                                                "count_until", 
                                                goal_callback = self.goal_callback, # Callback to accept the goal
                                                handle_accepted_callback = self.handle_accepted_callback, # Handle accepted goals callback
                                                cancel_callback = self.cancel_callback,   # Callback to attempt cancelling the callback
                                                execute_callback = self.execute_callback, # Callback to excecute the action
                                                callback_group=ReentrantCallbackGroup())  # Handling multple callbacks in different threads
        self.get_logger().info("Action server has been started")

    def goal_callback(self, goal_request: CountUntil.Goal):
        '''Callback to handle goal acceptance'''
        self.get_logger().info("Received a goal")

        # Policy: refuse new goal if current goal is active
        # with self.goal_lock_:
        #     if self.goal_handle_ and self.goal_handle_.is_active:
        #         self.get_logger().info("A goal is already active, rejecting new goal")
        #         return GoalResponse.REJECT

        # Validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT
        
        # Policy: preempt existing goal when receiving a new goal
        # with self.goal_lock_:
        #     # A current goal in execution
        #     if self.goal_handle_ and self.goal_handle_.is_active:
        #         self.get_logger().info("Abort current goal and accept new goal")
        #         self.goal_handle_.abort() # Abort current goal
        
        # Accept the goal request
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        '''Handle accepted goals callback'''
        with self.goal_lock_:
            if self.goal_handle_:
                self.goal_queue_.append(goal_handle) # Add the new goal to the queue
            else:
                # Execute goal if there is no goal active
                goal_handle.execute()
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        '''Callback to handle cancel requests'''
        self.get_logger().info("Received a cancel request")

        # Cancel request may be rejected due to different functional factors
        return CancelResponse.ACCEPT # or REJECT


    def execute_callback(self, goal_handle:ServerGoalHandle):
        '''Execute callback for action count_until'''
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute the action 
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0

        for _ in range(target_number):
            # Handling cancel requests

            counter += 1
            self.get_logger().info(str(counter))

            # Adding feedback during execution
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)

            # Wait for the next iteration
            sleep(period)

        # Once done, set goal final state 
        goal_handle.succeed()
        # goal_handle.abort()

        # send the result
        result.reached_number = counter

        self.process_next_goal()
        return result
    
    def process_next_goal(self):    
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute()

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServer() # MODIFY NAME
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()