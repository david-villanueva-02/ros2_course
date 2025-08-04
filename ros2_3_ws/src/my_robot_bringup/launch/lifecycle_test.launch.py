from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    ld = LaunchDescription()
    node_name: str = "my_number_publisher"

    number_node = LifecycleNode(
        package = "lifecycle_py",
        executable="number_publisher",
        name = node_name,
        namespace=""
    )

    lifecycle_node_manager_node = Node(
        package="lifecycle_py",
        executable="lifecycle_node_manager",
        parameters=[
            {"managed_node_name": node_name}
        ]
    )

    ld.add_action(number_node)
    ld.add_action(lifecycle_node_manager_node)
    
    return ld