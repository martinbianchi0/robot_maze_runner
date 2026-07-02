#!/usr/bin/env python3
"""Genera resumen y visuales de una corrida grabada con lab_record_all.sh."""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


Point = Tuple[float, float]


def read_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    with path.open(encoding='utf-8') as f:
        return json.load(f)


def read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def to_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    if value in (None, ''):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def parse_yaml_min(path: Path) -> Dict[str, Any]:
    meta: Dict[str, Any] = {}
    if not path.exists():
        return meta
    for raw in path.read_text(encoding='utf-8').splitlines():
        line = raw.strip()
        if not line or line.startswith('#') or ':' not in line:
            continue
        key, val = line.split(':', 1)
        val = val.strip()
        if val.startswith('['):
            meta[key.strip()] = [float(x.strip()) for x in val.strip('[]').split(',')]
        else:
            try:
                if any(c in val for c in '.eE'):
                    meta[key.strip()] = float(val)
                else:
                    meta[key.strip()] = int(val)
            except ValueError:
                meta[key.strip()] = val.strip('"\'')
    return meta


def read_pgm(path: Path) -> Tuple[int, int, int, bytes]:
    with path.open('rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f'{path} no es PGM P5')
        line = f.readline()
        while line.startswith(b'#') or not line.strip():
            line = f.readline()
        width, height = [int(x) for x in line.split()]
        maxval = int(f.readline())
        data = f.read(width * height)
    return width, height, maxval, data


def load_map(map_yaml: Path) -> Dict[str, Any]:
    meta = parse_yaml_min(map_yaml)
    if not meta:
        return {}
    image = Path(str(meta.get('image', '')))
    if not image.is_absolute():
        image = map_yaml.parent / image
    width, height, maxval, pixels = read_pgm(image)
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    return {
        'yaml': str(map_yaml),
        'image': str(image),
        'width': width,
        'height': height,
        'maxval': maxval,
        'pixels': pixels,
        'resolution': float(meta.get('resolution', 1.0)),
        'origin': (float(origin[0]), float(origin[1])),
        'negate': int(meta.get('negate', 0)),
        'occupied_thresh': float(meta.get('occupied_thresh', 0.65)),
        'free_thresh': float(meta.get('free_thresh', 0.25)),
    }


def map_probability(px: int, negate: int) -> float:
    return px / 255.0 if negate else (255.0 - px) / 255.0


def path_points(rows: Iterable[Dict[str, str]]) -> List[Point]:
    pts: List[Point] = []
    for row in rows:
        x = to_float(row.get('x'))
        y = to_float(row.get('y'))
        if x is not None and y is not None:
            pts.append((x, y))
    return pts


def pose_points(rows: Iterable[Dict[str, str]]) -> List[Tuple[float, float, float, float]]:
    pts = []
    for row in rows:
        t = to_float(row.get('t_wall'))
        x = to_float(row.get('x'))
        y = to_float(row.get('y'))
        yaw = to_float(row.get('yaw'), 0.0)
        if t is not None and x is not None and y is not None and yaw is not None:
            pts.append((t, x, y, yaw))
    return pts


def find_map_yaml(run_dir: Path, summary: Dict[str, Any], metadata: Dict[str, Any]) -> Optional[Path]:
    candidates = [
        summary.get('map_yaml'),
        metadata.get('map_yaml'),
        metadata.get('map'),
        'maps/laberinto_lab_20260702.yaml',
    ]
    roots = []
    if metadata.get('cwd'):
        roots.append(Path(str(metadata['cwd'])))
    roots.append(Path.cwd())
    for item in candidates:
        if not item:
            continue
        p = Path(str(item))
        if p.is_absolute() and p.exists():
            return p
        if not p.is_absolute():
            for root in roots:
                candidate = root / p
                if candidate.exists():
                    return candidate
    return None


def render_overlay_matplotlib(out_path: Path, map_data: Dict[str, Any], poses, goals,
                              latest_path, states, cones) -> bool:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception:
        return False

    if not map_data:
        return False
    w, h = map_data['width'], map_data['height']
    raw = np.frombuffer(map_data['pixels'], dtype=np.uint8).reshape(h, w)
    prob = raw.astype(np.float32) / 255.0 if map_data['negate'] else (255.0 - raw) / 255.0
    occ = prob > map_data['occupied_thresh']
    unknown = (prob >= map_data['free_thresh']) & (prob <= map_data['occupied_thresh'])
    occ = np.flipud(occ)
    unknown = np.flipud(unknown)

    res = map_data['resolution']
    ox, oy = map_data['origin']
    extent = [ox, ox + w * res, oy, oy + h * res]

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(unknown, origin='lower', cmap='Greys', extent=extent, alpha=0.20)
    ax.imshow(occ, origin='lower', cmap='Greys', extent=extent, alpha=0.85)

    if poses:
        xs = [p[1] for p in poses]
        ys = [p[2] for p in poses]
        ax.plot(xs, ys, color='#0b5cad', linewidth=2.0, label='trayectoria MCL')
        ax.scatter([xs[0]], [ys[0]], marker='s', s=70, color='#111111', label='inicio')
        ax.scatter([xs[-1]], [ys[-1]], marker='>', s=90, color='#0b5cad', label='final')

    if latest_path:
        ax.plot([p[0] for p in latest_path], [p[1] for p in latest_path],
                color='#c2185b', linewidth=2.0, linestyle='--', label='ultimo /plan')

    if goals:
        gx = [to_float(g.get('x')) for g in goals]
        gy = [to_float(g.get('y')) for g in goals]
        pts = [(x, y) for x, y in zip(gx, gy) if x is not None and y is not None]
        if pts:
            ax.scatter([p[0] for p in pts], [p[1] for p in pts], marker='X',
                       s=85, color='#2e7d32', label=f'goals ({len(pts)})')

    state_colors = {
        'RECOVERY': '#d32f2f',
        'AVOID_OBSTACLE': '#d32f2f',
        'REPLAN': '#f57c00',
        'FAILURE': '#000000',
        'IDLE': '#757575',
        'BLOCKED': '#d32f2f',
    }
    plotted_states = set()
    for row in states:
        state = row.get('state', '')
        color = state_colors.get(state)
        x = to_float(row.get('pose_x'))
        y = to_float(row.get('pose_y'))
        if color and x is not None and y is not None:
            label = state if state not in plotted_states else None
            ax.scatter([x], [y], marker='o', s=95, color=color, edgecolor='white',
                       linewidth=0.8, label=label)
            plotted_states.add(state)

    cone_rays = 0
    for row in cones:
        bearing = to_float(row.get('best_bearing_rad'))
        px = to_float(row.get('pose_x'))
        py = to_float(row.get('pose_y'))
        yaw = to_float(row.get('pose_yaw'))
        if bearing is None or px is None or py is None or yaw is None:
            continue
        if cone_rays > 80:
            continue
        length = 0.45
        ax.plot([px, px + length * math.cos(yaw + bearing)],
                [py, py + length * math.sin(yaw + bearing)],
                color='#ef6c00', alpha=0.35, linewidth=1.2)
        cone_rays += 1
    if cone_rays:
        ax.plot([], [], color='#ef6c00', alpha=0.8, label=f'detecciones cono ({cone_rays})')

    final_nav = next((row.get('state') for row in reversed(states)
                      if row.get('source') == 'nav'), '?')
    final_mission = next((row.get('state') for row in reversed(states)
                          if row.get('source') == 'mission'), '?')
    ax.set_title(f'Demo laboratorio - nav={final_nav} mission={final_mission}')
    ax.set_xlabel('x map [m]')
    ax.set_ylabel('y map [m]')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(alpha=0.18)
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return True


def world_to_svg(map_data: Dict[str, Any], x: float, y: float,
                 svg_w: int, svg_h: int) -> Tuple[float, float]:
    ox, oy = map_data['origin']
    w, h, res = map_data['width'], map_data['height'], map_data['resolution']
    sx = (x - ox) / (w * res) * svg_w
    sy = svg_h - (y - oy) / (h * res) * svg_h
    return sx, sy


def svg_polyline(map_data, pts: List[Point], svg_w, svg_h, color, width=2,
                 dash: str = '') -> str:
    if not pts:
        return ''
    coords = []
    for x, y in pts:
        sx, sy = world_to_svg(map_data, x, y, svg_w, svg_h)
        coords.append(f'{sx:.1f},{sy:.1f}')
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ''
    return (f'<polyline points="{" ".join(coords)}" fill="none" stroke="{color}" '
            f'stroke-width="{width}"{dash_attr} />\n')


def render_overlay_svg(out_path: Path, map_data: Dict[str, Any], poses, goals,
                       latest_path, states, cones) -> None:
    svg_w = 1000
    svg_h = 1000
    map_bits = []
    if map_data:
        w, h = map_data['width'], map_data['height']
        step = max(1, int(max(w, h) / 180))
        scale_x = svg_w / w
        scale_y = svg_h / h
        pixels = map_data['pixels']
        for y in range(0, h, step):
            for x in range(0, w, step):
                occ_seen = False
                unk_seen = False
                for yy in range(y, min(h, y + step)):
                    row = yy * w
                    for xx in range(x, min(w, x + step)):
                        p = map_probability(pixels[row + xx], map_data['negate'])
                        occ_seen = occ_seen or p > map_data['occupied_thresh']
                        unk_seen = unk_seen or map_data['free_thresh'] <= p <= map_data['occupied_thresh']
                if occ_seen or unk_seen:
                    color = '#111111' if occ_seen else '#d8d8d8'
                    map_bits.append(
                        f'<rect x="{x * scale_x:.1f}" y="{y * scale_y:.1f}" '
                        f'width="{step * scale_x + 0.5:.1f}" '
                        f'height="{step * scale_y + 0.5:.1f}" fill="{color}" '
                        f'fill-opacity="0.85" />')
    pose_pts = [(p[1], p[2]) for p in poses]
    state_colors = {'RECOVERY': '#d32f2f', 'AVOID_OBSTACLE': '#d32f2f',
                    'REPLAN': '#f57c00', 'FAILURE': '#000000',
                    'IDLE': '#757575', 'BLOCKED': '#d32f2f'}
    body = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_w}" height="{svg_h}" '
        f'viewBox="0 0 {svg_w} {svg_h}">',
        '<rect width="100%" height="100%" fill="white" />',
        *map_bits,
        svg_polyline(map_data, latest_path, svg_w, svg_h, '#c2185b', 3, '8 6') if map_data else '',
        svg_polyline(map_data, pose_pts, svg_w, svg_h, '#0b5cad', 4) if map_data else '',
    ]
    if map_data:
        for row in goals:
            x = to_float(row.get('x'))
            y = to_float(row.get('y'))
            if x is None or y is None:
                continue
            sx, sy = world_to_svg(map_data, x, y, svg_w, svg_h)
            body.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="8" fill="#2e7d32" />')
        for row in states:
            state = row.get('state', '')
            color = state_colors.get(state)
            x = to_float(row.get('pose_x'))
            y = to_float(row.get('pose_y'))
            if color and x is not None and y is not None:
                sx, sy = world_to_svg(map_data, x, y, svg_w, svg_h)
                body.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="10" fill="{color}" '
                            f'stroke="white" stroke-width="2"><title>{state}</title></circle>')
        cone_rays = 0
        for row in cones:
            bearing = to_float(row.get('best_bearing_rad'))
            px = to_float(row.get('pose_x'))
            py = to_float(row.get('pose_y'))
            yaw = to_float(row.get('pose_yaw'))
            if bearing is None or px is None or py is None or yaw is None or cone_rays > 80:
                continue
            x2 = px + 0.45 * math.cos(yaw + bearing)
            y2 = py + 0.45 * math.sin(yaw + bearing)
            sx1, sy1 = world_to_svg(map_data, px, py, svg_w, svg_h)
            sx2, sy2 = world_to_svg(map_data, x2, y2, svg_w, svg_h)
            body.append(f'<line x1="{sx1:.1f}" y1="{sy1:.1f}" x2="{sx2:.1f}" y2="{sy2:.1f}" '
                        'stroke="#ef6c00" stroke-width="2" stroke-opacity="0.35" />')
            cone_rays += 1
    body.append('<g font-family="sans-serif" font-size="22" fill="#111">'
                '<rect x="16" y="16" width="370" height="142" fill="white" fill-opacity="0.82" />'
                '<text x="30" y="48">azul: trayectoria MCL</text>'
                '<text x="30" y="80">verde: goals</text>'
                '<text x="30" y="112">magenta: ultimo /plan</text>'
                '<text x="30" y="144">naranja: detecciones cono</text></g>')
    body.append('</svg>')
    out_path.write_text('\n'.join(body), encoding='utf-8')


def write_timeline(run_dir: Path, states: List[Dict[str, str]], events: List[Dict[str, Any]]) -> None:
    path = run_dir / 'timeline.csv'
    rows = []
    for row in states:
        rows.append({
            't_wall': row.get('t_wall', ''),
            'kind': f"{row.get('source', '')}_state",
            'label': row.get('state', ''),
        })
    for ev in events:
        if ev.get('event') in ('goal_pose', 'plan_update', 'cone_detection'):
            rows.append({'t_wall': ev.get('t_wall', ''), 'kind': ev.get('event', ''),
                         'label': ev.get('state', ev.get('event', ''))})
    rows.sort(key=lambda r: to_float(r['t_wall'], 0.0) or 0.0)
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=['t_wall', 'kind', 'label'])
        writer.writeheader()
        writer.writerows(rows)


def read_events(path: Path, limit: int = 200000) -> List[Dict[str, Any]]:
    events = []
    if not path.exists():
        return events
    with path.open(encoding='utf-8') as f:
        for i, line in enumerate(f):
            if i >= limit:
                break
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return events


def write_summary_md(run_dir: Path, summary: Dict[str, Any], metadata: Dict[str, Any],
                     overlay_name: str, states: List[Dict[str, str]]) -> None:
    counts = summary.get('counts', {})
    final_nav = summary.get('last_nav_state') or '?'
    final_mission = summary.get('last_mission_state') or '?'
    duration = summary.get('duration_s')
    lines = [
        '# Demo laboratorio Parte C',
        '',
        f'- Directorio: `{run_dir}`',
        f'- Branch: `{metadata.get("branch", "?")}`',
        f'- Commit: `{metadata.get("commit", "?")}`',
        f'- Robot: `{metadata.get("ns", summary.get("ns", "?"))}`',
        f'- Mapa: `{metadata.get("map_yaml", summary.get("map_yaml", "?"))}`',
        f'- Duracion logger: `{duration:.1f} s`' if isinstance(duration, (int, float)) else '- Duracion logger: `?`',
        f'- Distancia estimada por MCL: `{summary.get("distance_m", 0.0):.2f} m`',
        f'- Estado final nav/mission: `{final_nav}` / `{final_mission}`',
        '',
        '## Archivos principales',
        '',
        f'- Rosbag: `rosbag/`',
        f'- Eventos: `events.jsonl`',
        f'- Timeline: `timeline.csv`',
        f'- Overlay: `{overlay_name}`',
        '',
        '## Conteos',
        '',
    ]
    for key in sorted(counts):
        lines.append(f'- `{key}`: {counts[key]}')
    lines.extend([
        f'- `recovery_events`: {summary.get("recovery_events", 0)}',
        f'- `idle_events`: {summary.get("idle_events", 0)}',
        '',
        '## Estados observados',
        '',
    ])
    if states:
        for row in states[-25:]:
            lines.append(f'- `{row.get("source")}` -> `{row.get("state")}` '
                         f'@ `{row.get("t_wall")}`')
    else:
        lines.append('- No se registraron cambios de estado.')
    lines.extend([
        '',
        '## Lectura rapida',
        '',
        '- Si aparecen puntos rojos/negros sobre la trayectoria, revisar `states.csv` '
        'y `events.jsonl` alrededor de esos tiempos.',
        '- Si hubo detecciones de cono sin goals, mirar si la FSM rechazo el punto '
        'por pared o por mapa inflado.',
        '- Si `/nav_debug` esta presente, `events.jsonl` contiene clearance frontal, '
        'cantidad de obstaculos dinamicos y razon del tick de control.',
        '',
    ])
    (run_dir / 'summary.md').write_text('\n'.join(lines), encoding='utf-8')


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    run_dir = args.run_dir
    if not run_dir.exists():
        print(f'No existe {run_dir}', file=sys.stderr)
        return 2

    summary = read_json(run_dir / 'summary.json')
    metadata = read_json(run_dir / 'metadata' / 'run.json')
    poses = pose_points(read_csv(run_dir / 'poses.csv'))
    goals = read_csv(run_dir / 'goals.csv')
    states = read_csv(run_dir / 'states.csv')
    cones = read_csv(run_dir / 'cone_detections.csv')
    latest_path = path_points(read_csv(run_dir / 'latest_path.csv'))
    events = read_events(run_dir / 'events.jsonl')

    write_timeline(run_dir, states, events)

    map_yaml = find_map_yaml(run_dir, summary, metadata)
    map_data = load_map(map_yaml) if map_yaml else {}
    overlay_name = 'map_overlay.png'
    overlay_path = run_dir / overlay_name
    ok = render_overlay_matplotlib(overlay_path, map_data, poses, goals, latest_path, states, cones)
    if not ok:
        overlay_name = 'map_overlay.svg'
        render_overlay_svg(run_dir / overlay_name, map_data, poses, goals, latest_path, states, cones)

    write_summary_md(run_dir, summary, metadata, overlay_name, states)
    print(f'[lab_make_report] escrito {run_dir / "summary.md"}')
    print(f'[lab_make_report] escrito {run_dir / overlay_name}')
    print(f'[lab_make_report] escrito {run_dir / "timeline.csv"}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
