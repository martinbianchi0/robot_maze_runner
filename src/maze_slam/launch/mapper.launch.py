"""Launch Etapa 1 - occupancy_mapper con pose conocida (/calc_odom)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_slam')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'maze_slam.rviz')

    open_rviz = LaunchConfiguration('rviz', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='maze_slam',
            executable='occupancy_mapper',
            name='occupancy_mapper',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'map_size_m': 16.0,
                'resolution': 0.05,
                'max_range': 3.5,
                'odom_topic': '/calc_odom',
                'scan_topic': '/scan',
                'map_topic': '/map',
                'map_frame': 'map',
                'publish_period_s': 1.0,
            }],
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_cfg],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
