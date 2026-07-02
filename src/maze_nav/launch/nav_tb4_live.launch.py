"""Parte B: pila de navegacion autonoma sobre el TB4 REAL (namespace tb4_0 / tb4_1).

Igual que nav.launch.py pero adaptado al robot real:
- use_sim_time=False (el TB4 no publica /clock).
- Sensores namespaced: /<ns>/scan, /<ns>/odom.
- Remapea /tf y /tf_static al /<ns>/tf del robot (RViz se arranca con el mismo
  remap). Si el localizer publicara map->odom en /tf global, RViz no lo ve y
  descarta el scan ("Message Filter dropping message").
- Remapea /cmd_vel -> /<ns>/cmd_vel (el TB4 escucha ahi; sin esto NO se mueve).
- LIDAR del TB4: montaje +90deg / -4cm. Velocidades conservadoras.

El localizer auto-detecta el frame del odom del msg ('odom' en el TB4), asi la
TF map->odom engancha con la cadena del robot (odom->base_link->rplidar_link).

Uso (Parte B: fijar pose y goal en RViz):
  ros2 launch maze_nav nav_tb4_live.launch.py \\
      map_yaml:=maps/laberinto_lab_20260702.yaml ns:=tb4_0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    ns = LaunchConfiguration('ns')
    v_max = LaunchConfiguration('v_max', default='0.12')
    w_max = LaunchConfiguration('w_max', default='0.8')
    # false por default (robot real); true si venis de un rosbag (publica /clock).
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    common = [{'use_sim_time': use_sim_time}]

    scan_topic = PythonExpression(["'/' + '", ns, "' + '/scan'"])
    odom_topic = PythonExpression(["'/' + '", ns, "' + '/odom'"])
    cmd_topic = PythonExpression(["'/' + '", ns, "' + '/cmd_vel'"])
    tf_topic = PythonExpression(["'/' + '", ns, "' + '/tf'"])
    tf_static_topic = PythonExpression(["'/' + '", ns, "' + '/tf_static'"])
    tf_remaps = [('/tf', tf_topic), ('/tf_static', tf_static_topic)]

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', description='mapa .yaml de la Parte A'),
        DeclareLaunchArgument('ns', default_value='tb4_0',
                              description='namespace del TB4 (tb4_0 / tb4_1)'),
        DeclareLaunchArgument('v_max', default_value='0.12'),
        DeclareLaunchArgument('w_max', default_value='0.8'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true si venis de un rosbag'),

        Node(
            package='maze_nav', executable='map_publisher', name='map_publisher',
            output='screen',
            parameters=common + [{'map_yaml': map_yaml}],
            remappings=tf_remaps,
        ),
        Node(
            package='maze_nav', executable='localizer', name='localizer',
            output='screen',
            parameters=common + [{
                'n_particles': 400,
                'sigma_hit': 0.20,
                'scan_topic': scan_topic,
                'odom_topic': odom_topic,
                'scan_yaw_offset': 1.5708,
                'scan_x_offset': -0.04,
            }],
            remappings=tf_remaps,
        ),
        Node(
            package='maze_nav', executable='navigator', name='navigator',
            output='screen',
            parameters=common + [{
                'v_max': v_max,
                'w_max': w_max,
                'lookahead': 0.35,
                'scan_topic': scan_topic,
                'scan_yaw_offset': 1.5708,
                'scan_x_offset': -0.04,
                # robot_radius+inflation = 0.20 (no 0.26): abre los pasillos
                # angostos del laberinto. DEBE coincidir con inflation_radius_m del
                # mission y --inflation del generador de waypoints (CONTEXTO_LABO.md).
                'robot_radius': 0.14,
                'inflation': 0.06,
            }],
            remappings=tf_remaps + [('/cmd_vel', cmd_topic)],
        ),
    ])
