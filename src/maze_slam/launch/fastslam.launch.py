"""Launch Etapa 2 - Grid-Based FastSLAM."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_slam')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'maze_slam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='maze_slam',
            executable='grid_fastslam',
            name='grid_fastslam',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'map_size_m': 16.0,
                'resolution': 0.05,
                'max_range': 3.5,
                'n_particles': 30,
                'odom_topic': '/calc_odom',
                'truth_topic': '/odom',
                'scan_topic': '/scan',
                'min_d_trans': 0.05,
                'min_d_rot': 0.05,
                'beam_step': 4,
                'sigma_hit': 0.07,
                'alpha1': 0.3,
                'alpha2': 0.05,
                'alpha3': 0.2,
                'alpha4': 0.05,
                'publish_period_s': 1.0,
            }],
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_cfg],
            output='screen',
        ),
    ])
