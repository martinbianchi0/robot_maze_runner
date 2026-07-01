"""Launch del detector de cono rojo.

Uso tipico:
  ros2 launch maze_perception cone_detector.launch.py \
      params_file:=/ruta/config/parte_c/bag.yaml

`params_file` es la ruta absoluta a un perfil (sim/bag/real). Los scripts de
smoke test pasan esa ruta. El perfil controla topicos, QoS de imagen, debug y
umbrales HSV. Si no se pasa params_file, el nodo arranca con sus defaults.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    has_params = PythonExpression(["'", params_file, "' != ''"])

    common = dict(package='maze_perception', executable='cone_detector',
                  name='cone_detector', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=''),
        Node(condition=IfCondition(has_params), parameters=[params_file], **common),
        Node(condition=UnlessCondition(has_params), **common),
    ])
