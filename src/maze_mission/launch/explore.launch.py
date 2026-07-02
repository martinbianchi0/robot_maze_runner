"""Pila de exploracion para laberinto DESCONOCIDO.

A diferencia de nav.launch.py (mapa estatico + MCL), aca el mapa lo construye
fastslam_node en vivo y la mision explora por fronteras. No se levanta map_publisher
ni localizer: fastslam_node publica /map y /amcl_pose.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maze_slam', executable='fastslam_node', name='fastslam_node',
            output='screen',
            parameters=[{'publish_tf': True}],
        ),
        Node(
            package='maze_nav', executable='navigator', name='navigator',
            output='screen',
        ),
        Node(
            package='maze_mission', executable='mission', name='mission_node',
            output='screen',
        ),
    ])
