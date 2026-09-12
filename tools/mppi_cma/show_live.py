"""Display RViz and a read-only dashboard while finite evaluations continue."""
from __future__ import annotations

import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from dashboard import server


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--display', default=':0')
    parser.add_argument('--xauthority', type=Path, required=True)
    parser.add_argument('--state-subdir', choices=('search', 'shared_course', 'shared_course_same_start'), default='search')
    args = parser.parse_args()
    root = args.root.resolve()
    gui = root / 'gui'
    gui.mkdir(exist_ok=True)
    lock = (gui / 'viewer.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if not args.xauthority.is_file():
        raise ValueError('Xauthority must be an existing file')
    os.environ.update(DISPLAY=args.display, XAUTHORITY=str(args.xauthority))
    httpd = server(root, state_subdir=args.state_subdir)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    image = json.loads((root / 'environment.json').read_text())['controller_image_id']
    spec = importlib.util.spec_from_file_location('cma_window_layout', gui / 'layout_sim_windows.py')
    layout = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = layout
    spec.loader.exec_module(layout)
    stopped = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    process, current, log, placed = None, None, None, False
    container = 'cma-mppi-rviz-viewer'
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
                    command = ['docker', 'run', '--rm', '--name', container,
                               '--network', 'container:' + target, '--gpus', 'all',
                               '-e', 'NVIDIA_DRIVER_CAPABILITIES=all', '-e', 'DISPLAY=' + args.display,
                               '-e', 'XAUTHORITY=/xauth', '-e', 'QT_X11_NO_MITSHM=1',
                               '-e', 'ROS_DOMAIN_ID=1', '-e', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
                               '-e', 'CYCLONEDDS_URI=file:///opt/autoware/cyclonedds.xml',
                               '-v', '/tmp/.X11-unix:/tmp/.X11-unix:ro',
                               '-v', str(args.xauthority) + ':/xauth:ro',
                               '-v', str(root / 'episodes' / active / 'cyclonedds.xml') + ':/opt/autoware/cyclonedds.xml:ro',
                               '-v', str(gui / 'live.rviz') + ':/viewer.rviz:ro',
                               image, 'rviz2', '-d', '/viewer.rviz', '--ros-args', '-p', 'use_sim_time:=true']
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                    current, placed = active, False
                    print('RViz following ' + active, flush=True)
            if process is not None and process.poll() is not None and not state['completed']:
                print('RViz exit: ' + str(process.returncode), flush=True)
                log.close()
                process, current = None, None
            if process is not None and not placed:
                windows = layout.list_client_windows()
                rviz = [w for w in windows if 'rviz' in w.title.lower()]
                browser = [w for w in windows if 'MPPI' in w.title and 'rviz' not in w.title.lower()]
                if rviz:
                    area = layout.active_workarea()
                    mover = layout.X11Mover()
                    try:
                        mover.place(rviz[-1].window_id, layout.Rect(area.x, area.y, area.width // 2, area.height))
                        if browser:
                            mover.place(browser[-1].window_id, layout.Rect(area.x + area.width // 2, area.y,
                                                                         area.width // 2, area.height))
                        placed = bool(browser)
                    finally:
                        mover.close()
            time.sleep(2)
    finally:
        httpd.shutdown()
        if process is not None:
            subprocess.run(['docker', 'stop', '-t', '2', container], capture_output=True, timeout=15)
            process.wait(timeout=10)
            log.close()


if __name__ == '__main__':
    main()
