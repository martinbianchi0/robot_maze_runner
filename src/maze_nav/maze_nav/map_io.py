"""Carga de mapas ROS YAML/PGM para navegacion."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class MapInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    frame_id: str = 'map'


def _read_pgm(path):
    """Lee P2/P5 PGM y devuelve una imagen uint8 top-down."""
    with Path(path).open('rb') as f:
        magic = f.readline().strip()
        if magic not in (b'P2', b'P5'):
            raise ValueError(f'Formato PGM no soportado en {path}: {magic!r}')

        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                raise ValueError(f'PGM incompleto: {path}')
            line = line.split(b'#', 1)[0]
            tokens.extend(line.split())

        width = int(tokens[0])
        height = int(tokens[1])
        max_value = int(tokens[2])
        if max_value <= 0 or max_value > 255:
            raise ValueError(f'Max value PGM no soportado: {max_value}')

        if magic == b'P5':
            data = np.frombuffer(f.read(width * height), dtype=np.uint8)
        else:
            rest = f.read().split()
            data = np.asarray([int(x) for x in rest[:width * height]], dtype=np.uint8)

    if data.size != width * height:
        raise ValueError(f'PGM con tamano inesperado: {path}')
    return data.reshape((height, width))


def image_to_occupancy(image, occupied_thresh=0.65, free_thresh=0.25, negate=0,
                       mode='trinary'):
    """Convierte imagen PGM de map_server a OccupancyGrid (-1/0/100).

    ROS guarda el PGM top-down. La grilla de navegacion se usa en coordenadas
    de mapa, con fila 0 como y minimo, por eso se invierte verticalmente.
    """
    image = np.asarray(image, dtype=np.float32)
    if negate:
        occ_prob = image / 255.0
    else:
        occ_prob = (255.0 - image) / 255.0

    occ_img = np.full(image.shape, -1, dtype=np.int8)
    occ_img[occ_prob > occupied_thresh] = 100
    occ_img[occ_prob < free_thresh] = 0
    if mode == 'trinary':
        # map_saver usa 205 para unknown en mapas trinary.
        occ_img[np.asarray(image, dtype=np.uint8) == 205] = -1
    return np.flipud(occ_img)


def load_map_yaml(yaml_path):
    """Carga un mapa YAML/PGM y devuelve (occupancy_grid, MapInfo)."""
    yaml_path = Path(yaml_path)
    if not yaml_path.is_absolute():
        yaml_path = Path.cwd() / yaml_path
    yaml_path = yaml_path.resolve()

    with yaml_path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    image_path = Path(data['image'])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    origin = data.get('origin', [0.0, 0.0, 0.0])
    image = _read_pgm(image_path)
    grid = image_to_occupancy(
        image,
        occupied_thresh=float(data.get('occupied_thresh', 0.65)),
        free_thresh=float(data.get('free_thresh', 0.25)),
        negate=int(data.get('negate', 0)),
        mode=str(data.get('mode', 'trinary')),
    )
    info = MapInfo(
        width=int(grid.shape[1]),
        height=int(grid.shape[0]),
        resolution=float(data['resolution']),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]) if len(origin) > 2 else 0.0,
    )
    return grid, info
