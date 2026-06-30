#!/usr/bin/env python3
"""Recupera el rosbag laberinto.

El db3 de la cátedra (`maps/laberinto/laberinto_0.db3`) tiene corrupción SQLite
intermitente. Leemos las filas crudas que se pueden (saltando páginas rotas con
bisección) y las re-escribimos con el writer OFICIAL de rosbag2 a formato MCAP,
que genera un bag limpio y bien finalizado que `ros2 bag play` acepta siempre.

Uso:
    python3 shs/recover_bag.py

Genera `maps/laberinto_fix/` (mcap + metadata.yaml).
Después `./shs/bag.sh` lo detecta y lo usa automáticamente.
"""

import sqlite3
import sys
from pathlib import Path

import rclpy  # noqa: F401
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialWriter,
    StorageOptions,
    TopicMetadata,
    get_registered_writers,
)
from rosidl_runtime_py.utilities import get_message


def main():
    here = Path(__file__).resolve().parent.parent
    src_db = here / 'maps' / 'laberinto' / 'laberinto_0.db3'
    dst_dir = here / 'maps' / 'laberinto_fix'

    if not src_db.exists():
        print(f'ERROR: no encuentro {src_db}', file=sys.stderr)
        sys.exit(1)
    # Siempre regeneramos desde cero (puede haber un dir parcial de un intento fallido).
    if dst_dir.exists():
        import shutil
        shutil.rmtree(dst_dir, ignore_errors=True)

    src = sqlite3.connect(f'file:{src_db}?mode=ro', uri=True)
    topics = src.execute(
        "SELECT id, name, type, serialization_format FROM topics"
    ).fetchall()
    topic_by_id = {tid: (name, typ, fmt) for tid, name, typ, fmt in topics}
    msg_cls = {typ: get_message(typ) for _, _, typ, _ in topics}
    end_guess = 196471

    # Elegir storage segun lo que ABRA en ESTE entorno. mcap es preferible (sobrevive
    # mejor en filesystems que corrompen sqlite grandes), pero el plugin de escritura
    # no siempre esta instalado. Probamos mcap y si no, sqlite3.
    available = set(get_registered_writers())
    conv = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    writer = None
    storage_id = None
    for cand in ('mcap', 'sqlite3'):
        if cand not in available:
            continue
        try:
            w = SequentialWriter()
            w.open(StorageOptions(uri=str(dst_dir), storage_id=cand), conv)
            writer, storage_id = w, cand
            break
        except Exception as e:  # noqa: BLE001
            print(f'  no pude usar {cand}: {e}', file=sys.stderr)
            # limpiar restos del intento fallido
            import shutil
            shutil.rmtree(dst_dir, ignore_errors=True)
    if writer is None:
        print('ERROR: ningun storage (mcap/sqlite3) disponible para escribir.', file=sys.stderr)
        sys.exit(1)
    print(f'Recuperando a formato: {storage_id}', flush=True)
    def make_topic_meta(tid, name, typ, fmt):
        # Jazzy: TopicMetadata(id, name, type, serialization_format, qos[list], hash)
        # Humble: TopicMetadata(name, type, serialization_format, qos='' [str])
        try:
            return TopicMetadata(id=tid, name=name, type=typ, serialization_format=fmt)
        except TypeError:
            return TopicMetadata(name=name, type=typ, serialization_format=fmt)

    for tid, name, typ, fmt in topics:
        writer.create_topic(make_topic_meta(tid, name, typ, fmt))

    cur = src.cursor()
    copied = 0
    skipped = 0

    def copy_range(lo, hi):
        nonlocal copied, skipped
        try:
            rows = cur.execute(
                f"SELECT topic_id, timestamp, data FROM messages "
                f"WHERE id BETWEEN {lo} AND {hi} ORDER BY id"
            ).fetchall()
            for topic_id, ts, data in rows:
                name, typ, _ = topic_by_id[topic_id]
                # Validamos deserializando+reserializando: descarta blobs basura.
                try:
                    msg = deserialize_message(bytes(data), msg_cls[typ])
                    writer.write(name, serialize_message(msg), int(ts))
                    copied += 1
                except Exception:
                    skipped += 1
        except sqlite3.DatabaseError:
            if lo == hi:
                skipped += 1
                return
            mid = (lo + hi) // 2
            copy_range(lo, mid)
            copy_range(mid + 1, hi)

    CHUNK = 2000
    print(f'Recuperando 1..{end_guess}...', flush=True)
    last = 0
    for lo in range(1, end_guess + 1, CHUNK):
        copy_range(lo, min(lo + CHUNK - 1, end_guess))
        if copied - last >= 20000:
            last = copied
            print(f'  {copied} ok, {skipped} skip', flush=True)

    del writer  # cierra y finaliza el bag (escribe metadata.yaml)
    src.close()
    # Marcador de "recuperacion completa". Guarda la distro de ROS que lo genero:
    # bag.sh confia en el bag si la distro coincide (sin re-validar caro cada vez).
    import os
    distro = os.environ.get('ROS_DISTRO', 'unknown')
    (dst_dir / '.recover_complete').write_text(
        f'distro={distro}\nstorage={storage_id}\nmsgs={copied}\n')
    print(f'\n{copied} mensajes recuperados ({storage_id}, {distro}), '
          f'{skipped} descartados -> {dst_dir}')


if __name__ == '__main__':
    main()
