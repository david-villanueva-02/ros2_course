import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimpleTFKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")

        # --------------- Static transform --------------------
        # Declare static broadcaster
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Static transform
        self.static_tf_stamped_ = TransformStamped()
        self.static_tf_stamped_.header.stamp = self.get_clock().now().to_msg()

        # Staticconnection
        self.static_tf_stamped_.header.frame_id = "bumperbot_base"
        self.static_tf_stamped_.child_frame_id = "bumperbot_top"

        # Translation vector
        self.static_tf_stamped_.transform.translation.x = 0.0
        self.static_tf_stamped_.transform.translation.y = 0.0
        self.static_tf_stamped_.transform.translation.z = 0.3

        # Rotation matrix
        # Quaternions instead of euler angles
        self.static_tf_stamped_.transform.rotation.x = 0.0 
        self.static_tf_stamped_.transform.rotation.y = 0.0
        self.static_tf_stamped_.transform.rotation.z = 0.0
        self.static_tf_stamped_.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(self.static_tf_stamped_)

        self.get_logger().info("Publishing static transform between %s and %s" % (self.static_tf_stamped_.header.frame_id, 
                                                                                  self.static_tf_stamped_.child_frame_id))

        # --------------- Dynamic transform --------------------
        # Declare dynamic broadcaster
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        # Dynamic transform
        self.dynamic_tf_stamped_ = TransformStamped()

        # Dynamic parametercs
        self.delta_x = 0.05 # Increment in x each iteration
        self.last_x = 0.0

        self.dymanic_timer = self.create_timer(0.1, self.dynamic_timer_callback)

    def dynamic_timer_callback(self):
        '''Publishes dynamic transform'''
        self.dynamic_tf_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_tf_stamped_.header.frame_id = "odom"
        self.dynamic_tf_stamped_.child_frame_id = "bumperbot_base"

        # Transform
        self.last_x = self.dynamic_tf_stamped_.transform.translation.x = self.last_x + self.delta_x
        self.dynamic_tf_stamped_.transform.translation.y = 0.0
        self.dynamic_tf_stamped_.transform.translation.z = 0.0

        # Rotation
        self.dynamic_tf_stamped_.transform.rotation.x = 0.0
        self.dynamic_tf_stamped_.transform.rotation.y = 0.0
        self.dynamic_tf_stamped_.transform.rotation.z = 0.0
        self.dynamic_tf_stamped_.transform.rotation.w = 1.0

        # Publish message
        self.dynamic_tf_broadcaster.sendTransform(self.dynamic_tf_stamped_)
        

def main():
    rclpy.init()
    simple_subscriber = SimpleTFKinematics()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown() 

if __name__ == "__main__":
    main()