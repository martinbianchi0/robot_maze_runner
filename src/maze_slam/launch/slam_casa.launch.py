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
                # Scan-match ON (igual que el laberinto, que anda impecable). El
                # /calc_odom de la casa integra la velocidad COMANDADA (no la real), asi
                # que deriva al girar -> con scan-match OFF el mapa sale blob. El tope
                # sm_max_ang limita la deriva rotacional en la sala simetrica. Si aun
                # asi girara, probar odom_topic:=/odom (ground truth) para mapa perfecto.
                'use_scan_match': True,
                'publish_tf': False,    # la catedra ya da map->odom en la casa
                'scan_topic': '/scan',
                'odom_topic': '/calc_odom',
                'odom_frame': 'calc_odom',
                'publish_rate': 4.0,
                'maps_dir': 'maps',
            }],
        ),
    ])
