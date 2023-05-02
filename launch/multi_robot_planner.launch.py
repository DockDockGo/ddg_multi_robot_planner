# my_launch_package/launch/map_subscriber_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the ROS2 package
    my_package_share_directory = get_package_share_directory("ddg_multi_robot_planner")

    # Create the ROS2 launch description
    ld = LaunchDescription()

    map_subscriber_node = Node(
        package="ddg_multi_robot_planner",
        executable="ddg_multi_robot_planner_node",
        output="screen",
    )

    # Add the node to the launch description
    ld.add_action(map_subscriber_node)

    return ld
