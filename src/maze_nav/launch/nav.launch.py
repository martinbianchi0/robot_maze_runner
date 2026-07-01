"""Parte B: pila de navegacion autonoma (localizacion + planning + control).

Asume que la simulacion de la casa ya esta corriendo (./shs/casa.sh). Levanta:
  - map_publisher: publica el mapa de la Parte A en /map
  - localizer:     MCL sobre /calc_odom + /scan + /map -> /amcl_pose (+ TF map->calc_odom)
  - navigator:     A* + pure-pursuit + FSM -> /cmd_vel

Argumentos:
  map_yaml:=/ruta/al/mapa.yaml   (mapa de la Parte A; obligatorio)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    common = [{'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', description='mapa .yaml de la Parte A'),

        Node(
            package='maze_nav', executable='map_publisher', name='map_publisher',
            output='screen',
            parameters=common + [{'map_yaml': map_yaml}],
        ),
        Node(
            package='maze_nav', executable='localizer', name='localizer',
            output='screen',
            parameters=common + [{
                'n_particles': 400,
                'sigma_hit': 0.15,
                'scan_topic': '/scan',
                'odom_topic': '/calc_odom',
            }],
        ),
        Node(
            package='maze_nav', executable='navigator', name='navigator',
            output='screen',
            parameters=common + [{
                'v_max': 0.18,
                'w_max': 1.2,
                'lookahead': 0.35,
                'scan_topic': '/scan',
            }],
        ),
    ])
