from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_alive_turtles_topic = ("alive_turtles", "old_turtles")

    map = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="closed_map",
        parameters=[{"background_r": 200},
                    {"background_g": 10},
                    {"background_b": 255}]
    )

    spawner = Node(
        package="follow_turtle_robot",
        executable="turtle_spawner",
        name="spawner_for_latest_app",
        parameters=[{"max_turtles": 5},
                    {"spawn_freq": 0.7}],
        remappings=[
            remap_alive_turtles_topic
        ]
    )

    robot = Node(
        package="follow_turtle_robot",
        executable="robot",
        name="robot_closed_target",
        parameters=[{'closed_turtle': False},
                    {"linear_speed": 1.5},
                    {"angular_speed": 15.0},
                    {"catch_distance": 0.9}],
        remappings=[
            remap_alive_turtles_topic
        ]
    )

    ld.add_action(map)
    ld.add_action(spawner)
    ld.add_action(robot)

    return ld