"""FastSLAM sobre la simulacion de la casa (TurtleBot3 en Gazebo).

A diferencia del bag TB4:
- Topics sin namespace: /scan y /calc_odom (odom ruidosa, consigna §1.3).
- LIDAR del TB3 centrado y sin rotar -> sensor_x/y/yaw = 0.
- publish_tf=False: la casa de la catedra ya publica map->odom estatico; nuestro
  nodo solo construye el mapa (usa los valores de /calc_odom internamente, no el TF).

Correr junto con la simulacion:
    T1: ./shs/casa.sh
    T2: ./shs/slam_casa.sh
    T3: ./shs/teleop.sh
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maze_slam',
            executable='fastslam_node',
            name='maze_slam',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'n_particles': 50,
                'resolution': 0.05,
                'map_size': 250,        # 250 * 0.05 = 12.5 m (alcanza la casa)
                'sigma_hit': 0.08,
                'alpha1': 0.04, 'alpha2': 0.02, 'alpha3': 0.05, 'alpha4': 0.02,
                # Scan-match APAGADO en la casa: el /calc_odom simulado es preciso, y
                # en la sala cuadrada (simetrica) el scan-match derivaba hacia una
                # alineacion rotada 90deg (el mapa salia girado). En el laberinto TB4 si
                # va activado porque alli la odometria real deriva y hay que corregirla.
                'use_scan_match': False,
                'publish_tf': False,    # la catedra ya da map->odom en la casa
                'scan_topic': '/scan',
                'odom_topic': '/calc_odom',
                'odom_frame': 'calc_odom',
                'publish_rate': 4.0,
                'maps_dir': 'maps',
            }],
        ),
    ])
