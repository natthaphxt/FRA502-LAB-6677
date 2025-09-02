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
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():

    # Messenger Node (for each turtle)
    turtle1sim_plus = Node(
        package="turtlesim_plus",
        executable="turtlesim_plus_node.py",
        name="turtlesim",
        output="screen",
    )

    eater = Node(
        package="lab2",
        executable="eater.py",
        name="eater",
        parameters=[{"rate": 5.0}],
        output="screen",
    )

    killer1 = Node(
        package="lab2",
        executable="killer.py",
        name="turtle_controller",
        namespace="killer1",  # Namespace for turtle2
        parameters=[
            {"rate": 10.0}],
        output="screen",
        
    )
    killer2 = Node(
        package="lab2",
        executable="killer.py",
        name="turtle2_controller",
        namespace="killer2",  # Namespace for turtle2
        parameters=[{"rate": 15.0}],
        output="screen",
    )
    odom_publisher = Node(
        package="lab2",
        executable="turtlesim_pose.py",
        name="odom_publisher",
        output="screen",
    )
    spawn = ExecuteProcess(
        cmd=[
        'ros2', 'service', 'call', '/spawn_turtle',
        'turtlesim/srv/Spawn',
        '{x: 1.0, y: 0.0, theta: 0.0, name: "turtle2"}'
        ],
        output='screen'
    )
    # crazy_pizza = Node(
    #     package="lab2",
    #     executable="crazy_pizza.py",
    #     name="crazy_pizza",
    #     output="screen",
    #     remappings=[("/cmd_vel", "/turtle1/cmd_vel")],
    # )
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
    launch_description.add_action(eater)
    launch_description.add_action(killer1)
    # launch_description.add_action(killer2)
    # launch_description.add_action(odom_publisher)
    # launch_description.add_action(rviz_node)
    # launch_description.add_action(crazy_pizza)

    return launch_description


def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()


if __name__ == "__main__":
    main()
