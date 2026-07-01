"""Launch de la maquina de estados de mision (Parte C).

Uso tipico:
  ros2 launch maze_mission mission.launch.py \
      params_file:=/ruta/config/parte_c/bag.yaml

`params_file` es la ruta absoluta a un perfil (sim/bag/real). Si no se pasa, el
nodo arranca con sus defaults. `mock_cones` se reserva para C5 (M3): inyectar
detecciones sinteticas para validar la FSM sin percepcion real.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    has_params = PythonExpression(["'", params_file, "' != ''"])

    common = dict(package='maze_mission', executable='mission',
                  name='mission_node', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('mock_cones', default_value='false'),
        Node(condition=IfCondition(has_params), parameters=[params_file], **common),
        Node(condition=UnlessCondition(has_params), **common),
    ])
