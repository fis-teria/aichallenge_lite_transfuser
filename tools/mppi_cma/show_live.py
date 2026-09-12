"""Display RViz and a read-only dashboard while finite evaluations continue."""
from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from dashboard import server
from continuation_state import study_path
from viewer_layout import VIEWER_CONFIG, VIEWER_CONTAINER, try_arrange


def clear_orphan_viewer(root: Path, image: str) -> None:
    """Reclaim only this study's RViz container while holding viewer.lock."""
    result = subprocess.run(['docker', 'inspect', VIEWER_CONTAINER],
                            capture_output=True, text=True, timeout=10)
    if result.returncode:
        if 'No such object' in result.stderr or 'No such container' in result.stderr:
            return
        raise RuntimeError(f'Cannot inspect RViz viewer: {result.stderr.strip()}')
    existing = json.loads(result.stdout)[0]
    config = existing['Config']
    viewer_files = {str(root / 'gui' / name) for name in ('live.rviz', 'shared.rviz')}
    owns_config = any(mount.get('Source') in viewer_files
                      and mount.get('Destination') in ('/viewer.rviz', VIEWER_CONFIG)
                      for mount in existing.get('Mounts', []))
    if (existing['Name'] != '/' + VIEWER_CONTAINER or config['Image'] != image
            or config.get('Cmd', [])[:1] != ['rviz2'] or not owns_config):
        raise RuntimeError('Existing RViz container does not belong to this study')
    print('Removing orphan RViz viewer', flush=True)
    subprocess.run(['docker', 'rm', '-f', existing['Id']], check=True,
                   capture_output=True, text=True, timeout=15)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--display', default=':0')
    parser.add_argument('--xauthority', type=Path, required=True)
    parser.add_argument('--state-subdir', default='search')
    args = parser.parse_args()
    root = args.root.resolve()
    study_path(root, args.state_subdir)
    gui = root / 'gui'
    gui.mkdir(exist_ok=True)
    lock = (gui / 'viewer.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if not args.xauthority.is_file():
        raise ValueError('Xauthority must be an existing file')
    os.environ.update(DISPLAY=args.display, XAUTHORITY=str(args.xauthority))
    image = json.loads((root / 'environment.json').read_text())['controller_image_id']
    clear_orphan_viewer(root, image)
    httpd = server(root, state_subdir=args.state_subdir)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    stopped = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    process, current, log, placed = None, None, None, False
    container = VIEWER_CONTAINER
    try:
        while not stopped:
            state = json.loads((root / args.state_subdir / 'state.json').read_text())
            active = state.get('active_episode')
            target = 'cma-mppi-' + active if active else None
            if active and active != current:
                exists = subprocess.run(['docker', 'inspect', target, '--format', '{{.State.Running}}'],
                                        capture_output=True, text=True)
                if exists.returncode == 0 and exists.stdout.strip() == 'true':
                    if process is not None:
                        subprocess.run(['docker', 'stop', '-t', '2', container], capture_output=True, timeout=15)
                        process.wait(timeout=10)
                        log.close()
                    log = (gui / ('rviz-' + active + '.log')).open('w')
                    config = gui / ('shared.rviz' if state.get('mode') == 'shared_course' else 'live.rviz')
                    if not config.exists():
                        config = gui / 'live.rviz'
                    command = ['docker', 'run', '--rm', '--name', container,
                               '--network', 'container:' + target, '--gpus', 'all',
                               '-e', 'NVIDIA_DRIVER_CAPABILITIES=all', '-e', 'DISPLAY=' + args.display,
                               '-e', 'XAUTHORITY=/xauth', '-e', 'QT_X11_NO_MITSHM=1',
                               '-e', 'ROS_DOMAIN_ID=1', '-e', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
                               '-e', 'CYCLONEDDS_URI=file:///opt/autoware/cyclonedds.xml',
                               '-v', '/tmp/.X11-unix:/tmp/.X11-unix:ro',
                               '-v', str(args.xauthority) + ':/xauth:ro',
                               '-v', str(root / 'episodes' / active / 'cyclonedds.xml') + ':/opt/autoware/cyclonedds.xml:ro',
                               '-v', str(config) + ':' + VIEWER_CONFIG + ':ro',
                               image, 'rviz2', '-d', VIEWER_CONFIG, '--ros-args', '-p', 'use_sim_time:=true']
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                    current, placed = active, False
                    print('RViz following ' + active, flush=True)
            if process is not None and process.poll() is not None and not state['completed']:
                print('RViz exit: ' + str(process.returncode), flush=True)
                log.close()
                process, current = None, None
            if process is not None and not placed:
                placed = try_arrange(gui)
            time.sleep(2)
    finally:
        httpd.shutdown()
        if process is not None:
            subprocess.run(['docker', 'stop', '-t', '2', container], capture_output=True, timeout=15)
            process.wait(timeout=10)
            log.close()


if __name__ == '__main__':
    main()
