"""Parte A live: FastSLAM sobre el TB4 REAL (namespace tb4_0 o tb4_1).

Igual que slam.launch.py (bag) pero adaptado al robot fisico:
- use_sim_time=False (el TB4 no publica /clock).
- Topicos scan/odom construidos con el argumento `ns`.
- Remapea /tf y /tf_static al /<ns>/tf del robot para que map->odom quede en
  el mismo bus que odom->base_link->rplidar_link (mismo criterio que
  nav_tb4_live.launch.py; sin esto RViz dropea el scan con "Message Filter").

El fastslam_node ya trae los defaults del TB4 correctos:
  sensor_x=-0.04, sensor_y=0, sensor_yaw=+pi/2 (rplidar_link a +90 deg).

Uso:
  ros2 launch maze_slam slam_tb4_live.launch.py ns:=tb4_0
  ros2 launch maze_slam slam_tb4_live.launch.py ns:=tb4_1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('ns')
    n_particles = LaunchConfiguration('n_particles', default='80')

    scan_topic = PythonExpression(["'/' + '", ns, "' + '/scan'"])
    odom_topic = PythonExpression(["'/' + '", ns, "' + '/odom'"])
    tf_topic = PythonExpression(["'/' + '", ns, "' + '/tf'"])
    tf_static_topic = PythonExpression(["'/' + '", ns, "' + '/tf_static'"])
    tf_remaps = [('/tf', tf_topic), ('/tf_static', tf_static_topic)]

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='tb4_0',
                              description='namespace del TB4 (tb4_0 / tb4_1)'),
        DeclareLaunchArgument('n_particles', default_value='80',
                              description='particulas del FastSLAM'),

        Node(
            package='maze_slam', executable='fastslam_node', name='maze_slam',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'n_particles': n_particles,
                'resolution': 0.03,
                'map_size': 500,
                'sigma_hit': 0.08,
                'alpha1': 0.04, 'alpha2': 0.02, 'alpha3': 0.05, 'alpha4': 0.02,
                'use_scan_match': True,
                'scan_topic': scan_topic,
                'odom_topic': odom_topic,
                # fastslam_node auto-corrige odom_frame con el frame_id que trae el msg.
                'odom_frame': 'odom',
                'publish_rate': 4.0,
                'maps_dir': 'maps',
                # save_map.sh sin argumento cae aca (en vez de casa_slam):
                'save_basename': 'maze_slam',
                # Montaje del LIDAR del TB4 (rplidar_link a -4 cm en X, +90 deg yaw).
                'sensor_x': -0.04,
                'sensor_y': 0.0,
                'sensor_yaw': 1.5707963,
            }],
            remappings=tf_remaps,
        ),
    ])
