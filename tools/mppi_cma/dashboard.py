"""Read-only live dashboard for an isolated MPPI CMA-ES study (standard library)."""
from __future__ import annotations

import argparse
import csv
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import time
if __package__:
    from .shared_course_state import native_vehicle_status
else:
    from shared_course_state import native_vehicle_status


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
            'vehicles': live.get('vehicles', []),
            'native_summary': live.get('summary'),
            'references_by_vehicle': {number: coordinates(Path(path)) for number, path in
                                      enumerate(config.get('vehicle_references', []), 1)},
            'candidate_names': [Path(path).parent.name for path in config.get('vehicle_references', [])],
            'reference': reference, 'target_mps': config.get('target_mps'),
            'handicap': config.get('handicap'), 'sample_age_s': age}


def snapshot(root: Path, state_subdir: str = 'search') -> dict:
    """Return progress and metre/second telemetry without changing experiment state."""
    if state_subdir not in ('search', 'shared_course', 'shared_course_same_start', 'refinement'):
        raise ValueError('Unknown study state directory')
    state = json.loads((root / state_subdir / 'state.json').read_text())
    shared = state.get('mode') == 'shared_course'
    active = state.get('active_episode')
    names = state.get('active_episodes', [active] if active else [])
    if shared and not names and state.get('last_episode'):
        names = [state['last_episode']]
    simulations = [episode_snapshot(root, name) for name in names]
    if shared and simulations:
        race = simulations[0]
        simulations = []
        for car in race['vehicles']:
            number = car['vehicle_number']
            metrics_path = root / 'episodes' / names[0] / f'metrics-d{number}.json'
            metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else {}
            simulations.append({**race, 'episode': f'{names[0]} · D{number}',
                                'vehicle_number': number, 'measured_median_mps': metrics.get('measured_speed_median_mps'),
                                'live': {k: car.get(k) for k in ('ego', 'command', 'status')}})
            simulations[-1]['live']['status'] = native_vehicle_status(race['native_summary'], number)
            if number in race['references_by_vehicle']:
                simulations[-1]['reference'] = race['references_by_vehicle'][number]
                simulations[-1]['episode'] = f"{race['candidate_names'][number-1]} · D{number}"
    focus = simulations[0] if simulations else episode_snapshot(root, None)
    conditions = {}
    for name, data in state.get('conditions', {}).items():
        records = [item['metrics'] for item in data['evaluations']]
        preferred = [item for item in records if item['preferred_feasible']]
        conditions[name] = {
            'evaluations': records,
            'candidate_count': sum('-g' in item['episode'] for item in records),
            'best': data['selected']['metrics'] if data.get('selected') else
                    min(preferred, key=lambda item: item['objective']) if preferred else None,
            'validated': data.get('validated_preferred_feasible'),
            'validation_count': len(data.get('comparison_runs', {}).get('candidate', data.get('best_repeats', []))),
            'comparison': data.get('comparison'),
        }
    calibration = json.loads((root / 'calibration.json').read_text())
    return {
        'completed': state['completed'], 'phase': state['phase'], 'active': active,
        'mode': state.get('mode', 'independent'), 'last_error': state.get('last_error'),
        'start_mode': state.get('start_mode'),
        'study_kind': state.get('study_kind'),
        'candidate_limit_per_condition': state.get('candidate_limit_per_condition', 18),
        'started_count': state['new_episodes_started'], 'maximum': state['maximum_new_episodes'],
        'conditions': conditions, 'live': focus['live'], 'simulations': simulations,
        'target_mps': focus['target_mps'], 'handicap': focus['handicap'],
        'sample_age_s': focus['sample_age_s'], 'reference': focus['reference'],
        'baseline': coordinates(root / 'snapshot/base_reference.csv'),
        'ot_polygon': calibration['ot_lane_polygon_map_m'],
    }


def server(root: Path, port: int = 8876, state_subdir: str = 'search') -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            try:
                if self.path.split('?', 1)[0] == '/api/state':
                    body = json.dumps(snapshot(root, state_subdir), allow_nan=False).encode()
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
    parser.add_argument('--state-subdir', choices=('search', 'shared_course', 'shared_course_same_start', 'refinement'), default='search')
    args = parser.parse_args()
    server(args.root, args.port, args.state_subdir).serve_forever()
