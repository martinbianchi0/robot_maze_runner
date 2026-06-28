import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_nav')
    config = LaunchConfiguration('config')
    rviz_config = PythonExpression([
        '"',
        os.path.join(pkg_share, 'rviz', 'nav_debug.rviz'),
        '" if "debug" == "',
        config,
        '" else "',
        os.path.join(pkg_share, 'rviz', 'nav_clean.rviz'),
        '"',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='clean'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
