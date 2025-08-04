#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import SetTarget
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from time import sleep
import threading

class SetTargetServer(LifecycleNode):
    def __init__(self):
        super().__init__("set_target_server")

        self.server_name = self.get_name()
        
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()

        # Initialize atrributes
        self.actiavted = False
        self.current_position: float = 50.0 # Initial robot position

        # Declare action 
        self.set_target_server_ = None

        self.get_logger().info("In constructor")

    # ----------- Lifecycle node management ----------- 

    def on_configure(self, previous_state: LifecycleState):
        # Action server creation
        self.get_logger().info("In on_configure")
        self.set_target_server_ = ActionServer(self, 
                                                SetTarget, 
                                                f"{self.server_name}/set_target", 
                                                goal_callback = self.goal_callback,         # Callback to accept the goal
                                                handle_accepted_callback = self.handle_accepted_callback, # Handle accepted goals callback
                                                cancel_callback = self.cancel_callback,     # Callback to attempt cancelling the callback
                                                execute_callback = self.execute_callback,   # Callback to excecute the action
                                                callback_group = ReentrantCallbackGroup(),  # Handling multple callbacks in different threads
                                                )
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, previous_state: LifecycleState):
        # Action server destruction
        self.get_logger().info("In on_cleanup")
        self.set_target_server_.destroy()
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("In on_activate")
        self.actiavted = True
        return super().on_activate(previous_state)
    
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("In on_deactivate")
        self.actiavted = False
        # Policy: preempt existing goal when receiving a new goal
        with self.goal_lock_:
            # A current goal in execution
            if self.goal_handle_ and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept new goal")
                self.goal_handle_.abort() # Abort current goal

        return super().on_deactivate(previous_state)
    
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("In on_shutdown")
        self.set_target_server_.destroy()
        return TransitionCallbackReturn.SUCCESS
    
    # ---------------- Functionalities ---------------- 

    def goal_callback(self, goal_request: SetTarget.Goal):
        '''Callback to handle goal acceptance'''
        self.get_logger().info("Received a goal")

        # Goal Request Validation
        if not self.actiavted: # Reject goal if the lifecycle node is not activated
            self.get_logger().warn("Node inactive")
            return GoalResponse.REJECT
        
        if not isinstance(goal_request.position, (int, float)) or not isinstance(goal_request.velocity, (int, float)):
            self.get_logger().warn("Wrong arguments type")
            return GoalResponse.REJECT

        if goal_request.position < 0 or goal_request.position > 100:
            self.get_logger().info("Position out of range, rejecting goal")
            return GoalResponse.REJECT
        
        if goal_request.position == self.current_position:
            self.get_logger().info("Robot in position")
            return GoalResponse.REJECT
        
        if goal_request.position < self.current_position: # Backwards
            if goal_request.velocity > 0:
                self.get_logger().info("Goal not reachable, rejecting goal")
                return GoalResponse.REJECT

        if goal_request.position > self.current_position: # Forward
            if goal_request.velocity < 0:
                self.get_logger().info("Goal not reachable, rejecting goal")
                return GoalResponse.REJECT

        # Policy: preempt existing goal when receiving a new goal
        with self.goal_lock_:
            # A current goal in execution
            if self.goal_handle_ and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept new goal")
                self.goal_handle_.abort() # Abort current goal
        
        # Accept the goal request
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        '''Handle accepted goals callback'''
        with self.goal_lock_:
            goal_handle.execute()
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        '''Callback to handle cancel requests'''
        self.get_logger().info("Received a cancel request")

        # Cancel request may be rejected due to different functional factors
        return CancelResponse.ACCEPT # or REJECT


    def execute_callback(self, goal_handle: ServerGoalHandle):
        '''Execute callback for action set_target'''
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        # Get request from goal
        position = goal_handle.request.position
        velocity = goal_handle.request.velocity

        # Execute the action 
        self.get_logger().info("Executing the goal")
        feedback = SetTarget.Feedback()
        result = SetTarget.Result()

        while self.current_position != position: # Until the robot reaches the target
            if not goal_handle.is_active: # Goal aborted
                result.position = self.current_position
                result.message = "Movement aborted"
                return result
                
            if goal_handle.is_cancel_requested: # Goal canceled
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled() # or succeed() or abort(), depending on functional factors
                result.position = self.current_position
                result.message = "Movement canceled"
                return result
            
            if abs(self.current_position - position) < abs(velocity):
                step = position - self.current_position
            else:
                step = velocity

            # Move
            self.current_position += step

            # Publish feedback
            feedback.current_position = self.current_position
            goal_handle.publish_feedback(feedback)

            # For debugging
            self.get_logger().info(f"Current position: {self.current_position}")

            # Assuming the velocity is in m/s
            sleep(1.0)

        # Once done, set goal final state 
        goal_handle.succeed()

        # send the result
        self.get_logger().info("Target reached")
        result.position = self.current_position
        result.message = "Target reached"

        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = SetTargetServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()