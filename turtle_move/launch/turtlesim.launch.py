import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = LaunchConfiguration("param_dir",
                default=os.path.join(get_package_share_directory("turtle_move"), "param", "turtlesim.yaml"))
    return LaunchDescription(
        [
            Node(package="turtlesim", executable="turtlesim_node", output="screen"),
            Node(package="turtle_move", executable="moveTurtleSim", output="screen"),
        ]
    )
