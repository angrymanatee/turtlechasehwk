"""Basic Turtle Chase Launch File
"""


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    hero_name = "turtle1"

    ld.add_action(Node(
        name="turtlesim",
        package="turtlesim",
        executable="turtlesim_node",
    ))

    ld.add_action(Node(
        name="villain_pool",
        package="turtlechasehwk",
        executable="villain_pool",
        parameters=[
            {"spawn_frequency": 0.1},
        ],
    ))

    ld.add_action(Node(
        name="hero_turtle",
        package="turtlechasehwk",
        executable="hero_turtle",
        parameters=[
            {"hero_name": hero_name},
            {"capture_dist": 0.5},
            {"control/pos/P": 1.0},
            {"control/pos/min": 1.0},
            {"control/pos/max": 10.0},
            {"control/ang/P": 10.0},
            {"control/ang/min": -10.0},
            {"control/ang/max": 10.0},
        ],
    ))

    return ld
