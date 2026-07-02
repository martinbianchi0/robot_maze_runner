"""Parte B: pila de navegacion autonoma (localizacion + planning + control).

Asume que la simulacion de la casa ya esta corriendo (./shs/casa.sh). Levanta:
  - map_publisher: publica el mapa de la Parte A en /map
  - localizer:     MCL sobre /calc_odom + /scan + /map -> /amcl_pose (+ TF map->calc_odom)
  - navigator:     A* + pure-pursuit + FSM -> /cmd_vel

Argumentos:
  map_yaml:=/ruta/al/mapa.yaml   (mapa de la Parte A; obligatorio)

Perfil sim (defaults): scan alineado al frente, offset TB3 burger.
Perfil ROBOT REAL (TurtleBot4, ver INTERFAZ_MAZE_NAV.md):
  ros2 launch maze_nav nav.launch.py map_yaml:=maps/maze_slam.yaml \
    use_sim_time:=false scan_topic:=/tb4_0/scan odom_topic:=/tb4_0/odom \
    scan_yaw_offset:=1.5708 scan_x_offset:=-0.04 v_max:=0.12 w_max:=0.8
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    odom_topic = LaunchConfiguration('odom_topic', default='/calc_odom')
    # Montaje del LIDAR: TB3 sim = (0.0, -0.032); TB4 real = (1.5708, -0.04).
    scan_yaw_offset = LaunchConfiguration('scan_yaw_offset', default='0.0')
    scan_x_offset = LaunchConfiguration('scan_x_offset', default='-0.032')
    v_max = LaunchConfiguration('v_max', default='0.18')
    w_max = LaunchConfiguration('w_max', default='1.2')

    common = [{'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', description='mapa .yaml de la Parte A'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/calc_odom'),
        DeclareLaunchArgument('scan_yaw_offset', default_value='0.0',
                              description='montaje angular del LIDAR (rad); TB4 real: 1.5708'),
        DeclareLaunchArgument('scan_x_offset', default_value='-0.032',
                              description='montaje lineal del LIDAR (m); TB4 real: -0.04'),
        DeclareLaunchArgument('v_max', default_value='0.18'),
        DeclareLaunchArgument('w_max', default_value='1.2'),

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
                'scan_topic': scan_topic,
                'odom_topic': odom_topic,
                'scan_yaw_offset': scan_yaw_offset,
                'scan_x_offset': scan_x_offset,
            }],
        ),
        Node(
            package='maze_nav', executable='navigator', name='navigator',
            output='screen',
            parameters=common + [{
                'v_max': v_max,
                'w_max': w_max,
                'lookahead': 0.35,
                'scan_topic': scan_topic,
                'scan_yaw_offset': scan_yaw_offset,
                'scan_x_offset': scan_x_offset,
            }],
        ),
    ])
