"""Publica el mapa estatico de la Parte A en /map (latched).

Reemplazo minimo de nav2_map_server para no arrastrar toda la pila de Nav2.
Lee un .yaml/.pgm (formato map_server) y publica un OccupancyGrid con QoS
transient_local, para que quien se suscriba despues igual lo reciba.
"""
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

from maze_nav.nav_utils import load_map


def latched_qos():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('frame_id', 'map')
        yaml_path = self.get_parameter('map_yaml').value
        self.frame = self.get_parameter('frame_id').value

        if not yaml_path or not os.path.isfile(yaml_path):
            self.get_logger().error(
                f'map_yaml invalido: "{yaml_path}". Pasá -p map_yaml:=/ruta/al/mapa.yaml')
            raise SystemExit(1)

        m = load_map(yaml_path)
        self.get_logger().info(
            f'Mapa cargado: {m["W"]}x{m["H"]} @ {m["res"]}m, origin={m["origin"]} ({yaml_path})')

        grid = OccupancyGrid()
        grid.header.frame_id = self.frame
        grid.info.resolution = m['res']
        grid.info.width = m['W']
        grid.info.height = m['H']
        grid.info.origin.position.x = m['origin'][0]
        grid.info.origin.position.y = m['origin'][1]
        grid.info.origin.orientation.w = 1.0
        grid.data = m['occ'].astype('int8').flatten().tolist()
        self.grid = grid

        self.pub = self.create_publisher(OccupancyGrid, '/map', latched_qos())
        self.timer = self.create_timer(2.0, self._publish)
        self._publish()

    def _publish(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.grid)


def main():
    rclpy.init()
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
