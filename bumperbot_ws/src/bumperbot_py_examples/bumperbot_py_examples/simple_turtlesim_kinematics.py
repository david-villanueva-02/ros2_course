import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from math import pi, cos, sin

class SimpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__("simple_turtlesim_kinematics")

        self.turtle1_pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.turtle1_pose_callback, 10)
        self.turtle2_pose_subscriber = self.create_subscription(Pose, "/turtle2/pose", self.turtle2_pose_callback, 10)

        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

        self.translation_vector = [0, 0]

    def turtle1_pose_callback(self, msg: Pose):
        self.last_turtle1_pose_ = msg

    def turtle2_pose_callback(self, msg: Pose):
        self.last_turtle2_pose_ = msg

        # Translation 
        self.translation_vector[0] = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        self.translation_vector[1] = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y
        
        # Rotation (rad)
        theta_rad = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        theta_deg = theta_rad*180/pi # in degrees


        self.get_logger().info(f"""Translation vector: 
                               \ndelta_x :{self.translation_vector[0]}
                               \ndelta_y: {self.translation_vector[1]}
                               Rotation Matrix:
                               \ndelta_theta: {theta_deg}\n
                               |R11       R12| : |{cos(theta_rad)}     {-sin(theta_rad)}|\n
                               |R21       R22| : |{sin(theta_rad)}     {cos(theta_rad)}|\n
                               """)

def main():
    rclpy.init()
    simple_turtlesim_kinematics = SimpleTurtlesimKinematics()
    rclpy.spin(simple_turtlesim_kinematics)
    simple_turtlesim_kinematics.destroy_node()
    rclpy.shutdown() 

if __name__ == "__main__":
    main()