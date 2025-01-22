from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("number", "numero")

    number_publisher_node = Node(
        package='my_py_pkg',
        executable='number_publisher',
        name="THE_number_publisher",
        remappings=[
            remap_number_topic
        ],
        parameters=[
            {"number": 1},
            {"period": 0.001}
        ]
    )

    number_counter_node = Node(
        package='my_py_pkg',
        executable='number_counter',
        name="THE_number_counter",
        remappings=[
            remap_number_topic,
            ("number_counter", "contador_numero")
        ]
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)

    return ld