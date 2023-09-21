from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_alive_turtles_topic = ("alive_turtles", "dangerous_turtles")

    map = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="closed_map",
        parameters=[{"background_r": 255},
                    {"background_g": 255},
                    {"background_b": 70}]
    )

    spawner = Node(
        package="follow_turtle_robot",
        executable="turtle_spawner",
        name="spawner_for_closed_app",
        parameters=[{"max_turtles": 15},
                    {"spawn_freq": 1.5}],
        remappings=[
            remap_alive_turtles_topic
        ]
    )

    robot = Node(
        package="follow_turtle_robot",
        executable="robot",
        name="robot_closed_target",
        parameters=[{"closed_turtle": True},
                    {"linear_speed": 1.0},
                    {"angular_speed": 2.0},
                    {"catch_distance": 0.6}],
        remappings=[
            remap_alive_turtles_topic
        ]
    )

    ld.add_action(map)
    ld.add_action(spawner)
    ld.add_action(robot)

    return ld