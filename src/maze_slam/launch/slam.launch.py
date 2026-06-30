"""Lanza el nodo de FastSLAM contra el rosbag del laberinto del TB4.

Topics del bag (`maps/laberinto/laberinto_0.db3`):
    /tb4_0/scan   sensor_msgs/LaserScan
    /tb4_0/odom   nav_msgs/Odometry
    /tb4_0/tf     tf2_msgs/TFMessage
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
                'use_sim_time': True,   # el bag publica /clock; sin esto, TF queda fuera de rango
                # Config "C" (la que dio el mejor mapa: consist 0.023m). 80 particulas
                # + res fina + scan-match. Con numba corre ~45ms/update -> llega en vivo.
                # Mas particulas = filtro mas estable = el mapa NO salta de orientacion.
                # (El "exit -9" de antes era el build corrupto en la raid5, ya resuelto
                #  con el build en disco local; no era memoria.)
                'n_particles': 80,
                'resolution': 0.03,     # celdas finas -> paredes nitidas
                'map_size': 500,        # 500 * 0.03 = 15 m
                'sigma_hit': 0.08,
                'alpha1': 0.04, 'alpha2': 0.02, 'alpha3': 0.05, 'alpha4': 0.02,
                'use_scan_match': True,
                'odom_topic': '/tb4_0/odom',
                'scan_topic': '/tb4_0/scan',
                'odom_frame': 'odom',   # se auto-corrige si el msg trae otro frame_id
                'publish_rate': 4.0,
                'maps_dir': 'maps',
            }],
        ),
    ])
