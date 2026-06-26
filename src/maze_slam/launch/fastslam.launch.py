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
                # Scan-matching local: implementado y funcional unit-test, pero
                # la version con ref_dt compartida diverge en sim al colapsar
                # diversidad de particulas. Default OFF — para activar hacen
                # falta per-particle DT o improved proposal con covarianza
                # estimada (Grisetti). Ver issue/TODO en grid_fastslam.py.
                'scan_match': False,
                'match_win_xy': 0.10,
                'match_step_xy': 0.02,
                'match_win_th_deg': 3.0,
                'match_step_th_deg': 1.0,
                'match_reg_xy': 50.0,
                'match_reg_th': 50000.0,
                'match_min_occ': 500,
                'min_range': 0.12,
                'backend': 'auto',
                'publish_period_s': 1.0,
            }],
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_cfg],
            output='screen',
        ),
    ])
