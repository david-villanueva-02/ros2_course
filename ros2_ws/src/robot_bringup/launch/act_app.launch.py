from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    names = ("gerardo", "ola2", "caca", "udem", "robotsito")

    for name in names: 
        nodo = Node(
            package='my_py_pkg',
            executable='robot_news_station',
            name="robot_news_station_" + name,
            parameters=[
                {"robot_name": name}
            ]
        )
        ld.add_action(nodo)

    smartphone_node = Node(
        package='my_py_pkg',
        executable='smartphone',
        name="smartphone"
    )
    ld.add_action(smartphone_node)

    return ld