from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_mode = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name="main_turtle",
    )

    turtle_manager_node = Node(
        package='catch_them_all',
        executable='turtle_manager',
    )

    turtle_killer_node = Node(
        package='catch_them_all',
        executable='killer_controller',
        parameters=[
            {"k_gain_theta": 9.0}
        ]
    )

    turtle_spawner_node = Node(
        package='catch_them_all',
        executable='turtle_spawner',
        parameters=[
            {"spawning_period": 0.75}
        ]
    )

    ld.add_action(turtlesim_mode)
    ld.add_action(turtle_manager_node)
    ld.add_action(turtle_killer_node)
    ld.add_action(turtle_spawner_node)

    return ld