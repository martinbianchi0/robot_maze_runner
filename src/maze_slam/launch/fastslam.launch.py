"""Launch Etapa 2 - Grid-Based FastSLAM."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_slam')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'maze_slam.rviz')

    # Args para iterar sin editar el archivo. Defaults = comportamiento previo
    # (no cambia nada para quien lo lanza tal cual). En Mac/CPU conviene bajar
    # n_particles (ej: n_particles:=6) porque el raycasting es Python puro.
    n_particles = ParameterValue(LaunchConfiguration('n_particles'), value_type=int)
    scan_match = ParameterValue(LaunchConfiguration('scan_match'), value_type=bool)
    per_particle_dt = ParameterValue(LaunchConfiguration('per_particle_dt'), value_type=bool)
    backend = LaunchConfiguration('backend')
    # Ruido del motion model. Con /calc_odom bueno conviene bajarlos para que las
    # particulas no se dispersen y el mapa no "rote" (ej: alpha1:=0.05 ...).
    alpha1 = ParameterValue(LaunchConfiguration('alpha1'), value_type=float)
    alpha2 = ParameterValue(LaunchConfiguration('alpha2'), value_type=float)
    alpha3 = ParameterValue(LaunchConfiguration('alpha3'), value_type=float)
    alpha4 = ParameterValue(LaunchConfiguration('alpha4'), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('n_particles', default_value='30'),
        DeclareLaunchArgument('scan_match', default_value='false'),
        DeclareLaunchArgument('per_particle_dt', default_value='false'),
        DeclareLaunchArgument('backend', default_value='auto'),
        DeclareLaunchArgument('alpha1', default_value='0.3'),
        DeclareLaunchArgument('alpha2', default_value='0.05'),
        DeclareLaunchArgument('alpha3', default_value='0.2'),
        DeclareLaunchArgument('alpha4', default_value='0.05'),
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
                'n_particles': n_particles,
                'odom_topic': '/calc_odom',
                'truth_topic': '/odom',
                'scan_topic': '/scan',
                'min_d_trans': 0.05,
                'min_d_rot': 0.05,
                'beam_step': 4,
                'sigma_hit': 0.07,
                'alpha1': alpha1,
                'alpha2': alpha2,
                'alpha3': alpha3,
                'alpha4': alpha4,
                # Scan-matching local: implementado y funcional unit-test, pero
                # la version con ref_dt compartida diverge en sim al colapsar
                # diversidad de particulas. Default OFF — para activar hacen
                # falta per-particle DT o improved proposal con covarianza
                # estimada (Grisetti). Ver issue/TODO en grid_fastslam.py.
                'scan_match': scan_match,
                'per_particle_dt': per_particle_dt,
                'match_win_xy': 0.10,
                'match_step_xy': 0.02,
                'match_win_th_deg': 3.0,
                'match_step_th_deg': 1.0,
                'match_reg_xy': 50.0,
                'match_reg_th': 50000.0,
                'match_min_occ': 500,
                'min_range': 0.12,
                'backend': backend,
                # GPU prefereida en esta workstation: RTX 5070 (CUDA idx 1).
                # Verificar mapping con nvidia-smi --query-gpu=index,name --format=csv
                # vs el orden de CUDA (por compute capability). Cambiar a -1 para
                # que CUDA elija default, o al idx que corresponda en tu equipo.
                'gpu_device': 1,
                'gpu_mem_limit_gb': 9.6,
                'publish_period_s': 1.0,
            }],
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_cfg],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
