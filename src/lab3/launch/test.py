import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():

    # Turtlesim Plus Node
    turtle1sim_plus = Node(
        package="turtlesim_plus",
        executable="turtlesim_plus_node.py",
        name="turtlesim",
        output="screen",
    )

    # Pre-spawn turtle2 before killer1 starts
    spawn_turtle2 = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/spawn_turtle",
                    "turtlesim/srv/Spawn",
                    '{x: 2.0, y: 2.0, theta: 0.0, name: "turtle2"}',
                ],
                output="screen",
            )
        ],
    )

    # Pre-spawn turtle3 before killer2 starts
    spawn_turtle3 = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/spawn_turtle",
                    "turtlesim/srv/Spawn",
                    '{x: 8.0, y: 8.0, theta: 0.0, name: "turtle3"}',
                ],
                output="screen",
            )
        ],
    )

    # Eater Node
    eater = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="lab2",
                executable="eater.py",
                name="eater",
                parameters=[{"rate": 5.0}],
                output="screen",
            )
        ],
    )

    # First Killer - starts after turtle2 exists
    killer1 = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="lab2",
                executable="killer.py",
                name="turtle_controller",
                namespace="killer1",
                parameters=[{"rate": 10.0}],
                output="screen",
            )
        ],
    )

    # Second Killer - starts after turtle3 exists, with remapping
    killer2 = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="lab2",
                executable="killer.py",
                name="turtle_controller",
                namespace="killer2",
                parameters=[{"rate": 15.0}],
                output="screen",
                remappings=[
                    ("/turtle2/cmd_vel", "/turtle3/cmd_vel"),
                    ("/turtle2/pose", "/turtle3/pose"),
                ],
            )
        ],
    )

    # Odometry Publisher
    odom_publisher = Node(
        package="lab2",
        executable="turtlesim_pose.py",
        name="odom_publisher",
        output="screen",
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("turtle_pkg"), "rviz", "fun2.rviz"
            ),
        ],
        output="screen",
    )

    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(turtle1sim_plus)
    # launch_description.add_action(spawn_turtle2)
    # launch_description.add_action(spawn_turtle3)
    launch_description.add_action(eater)
    launch_description.add_action(killer1)
    launch_description.add_action(killer2)
    launch_description.add_action(odom_publisher)
    launch_description.add_action(rviz_node)

    return launch_description


def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        sys.exit()


if __name__ == "__main__":
    main()
