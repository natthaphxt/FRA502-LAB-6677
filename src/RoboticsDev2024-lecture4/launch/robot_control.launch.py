from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro


def generate_launch_description():

    pkg = get_package_share_directory("example_description")
    rviz_path = os.path.join(pkg, "config", "final.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_path],
        output="screen",
    )
    path_description = os.path.join(pkg, "robot", "visual", "my-robot.xacro")
    robot_desc_xml = xacro.process_file(path_description).toxml()
    parameters = [{"robot_description": robot_desc_xml}]
    robot_state_publisher = Node(   
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=parameters,
    )
    controller = Node(package="example_description", executable="controller.py")
    random = Node(package="example_description", executable="random_target.py")
    scheduler = Node(package="example_description", executable="scheduler.py")

    launch_description = LaunchDescription()

    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(scheduler)
    launch_description.add_action(random)
    launch_description.add_action(controller)

    return launch_description
