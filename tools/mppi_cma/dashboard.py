"""Read-only live dashboard for an isolated MPPI CMA-ES study (standard library)."""
from __future__ import annotations

import argparse
import csv
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import time


def last_sample(path: Path) -> dict:
    """Read the latest complete JSONL record, allowing an in-progress final write."""
    if not path.exists():
        return {}
    with path.open('rb') as stream:
        stream.seek(max(0, path.stat().st_size - 262144))
        lines = stream.read().split(b'\n')
    for line in reversed(lines[:-1]):
        try:
            return json.loads(line)
        except (json.JSONDecodeError, UnicodeDecodeError):
            continue
    return {}


def coordinates(path: Path) -> list[list[float]]:
    with path.open() as stream:
        return [[float(row['x_m']), float(row['y_m'])] for row in csv.DictReader(stream)]


def episode_snapshot(root: Path, active: str | None) -> dict:
    live, reference, config, age = {}, [], {}, None
    if active:
        episode = root / 'episodes' / active
        sample_path = episode / 'samples.jsonl'
        live = last_sample(sample_path)
        if sample_path.exists():
            age = time.time() - sample_path.stat().st_mtime
        if (episode / 'config.json').exists():
            config = json.loads((episode / 'config.json').read_text())
        command_path = episode / 'docker_command.json'
        if command_path.exists():
            command = json.loads(command_path.read_text())
            for arg in command:
                if ':/aichallenge/workspace/install/multi_purpose_mpc_ros/share/' in arg:
                    reference = coordinates(Path(arg.split(':', 1)[0]))
    return {'episode': active, 'live': {k: live.get(k) for k in ('ego', 'command', 'status', 'admin')},
            'reference': reference, 'target_mps': config.get('target_mps'),
            'handicap': config.get('handicap'), 'sample_age_s': age}


def snapshot(root: Path) -> dict:
    """Return progress and metre/second telemetry without changing experiment state."""
    state = json.loads((root / 'search/state.json').read_text())
    active = state.get('active_episode')
    names = state.get('active_episodes', [active] if active else [])
    simulations = [episode_snapshot(root, name) for name in names]
    focus = simulations[0] if simulations else episode_snapshot(root, None)
    conditions = {}
    for name, data in state.get('conditions', {}).items():
        records = [item['metrics'] for item in data['evaluations']]
        preferred = [item for item in records if item['preferred_feasible']]
        conditions[name] = {
            'evaluations': records,
            'candidate_count': sum('-g' in item['episode'] for item in records),
            'best': min(preferred, key=lambda item: item['objective']) if preferred else None,
            'validated': data.get('validated_preferred_feasible'),
            'validation_count': len(data.get('best_repeats', [])),
        }
    calibration = json.loads((root / 'calibration.json').read_text())
    return {
        'completed': state['completed'], 'phase': state['phase'], 'active': active,
        'started_count': state['new_episodes_started'], 'maximum': state['maximum_new_episodes'],
        'conditions': conditions, 'live': focus['live'], 'simulations': simulations,
        'target_mps': focus['target_mps'], 'handicap': focus['handicap'],
        'sample_age_s': focus['sample_age_s'], 'reference': focus['reference'],
        'baseline': coordinates(root / 'snapshot/base_reference.csv'),
        'ot_polygon': calibration['ot_lane_polygon_map_m'],
    }


def server(root: Path, port: int = 8876) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            try:
                if self.path.split('?', 1)[0] == '/api/state':
                    body = json.dumps(snapshot(root), allow_nan=False).encode()
                    mime = 'application/json; charset=utf-8'
                elif self.path in ('/', '/index.html'):
                    body = Path(__file__).with_name('dashboard.html').read_bytes()
                    mime = 'text/html; charset=utf-8'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', mime)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (OSError, ValueError, KeyError) as exc:
                self.send_error(503, str(exc))

        def log_message(self, _format: str, *args: object) -> None:
            return

    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--port', type=int, default=8876)
    args = parser.parse_args()
    server(args.root, args.port).serve_forever()
