import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("maze_slam")
    config_path = os.path.join(package_share, "config", "known_pose_mapping.yaml")

    return LaunchDescription(
        [
            Node(
                package="maze_slam",
                executable="known_pose_mapper",
                name="known_pose_mapper",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
