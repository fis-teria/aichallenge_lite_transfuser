"""Convenient finite AWSIM launch on the prepared graneple host deployment."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import re
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from aic_transfuser_lite.control.time_dev_v1 import TimeDevSpeeds


def make_command(*, source: Path, deployment: Path, run_id: str, display: str,
                 speeds: TimeDevSpeeds, record_video: bool) -> list[str]:
    """Explicit argument vector, no shell interpolation; outer timeout in seconds."""
    if not re.fullmatch(r'codex-time-[a-z0-9-]+', run_id):
        raise ValueError('INVALID_OWNED_RUN_ID')
    if not re.fullmatch(r':[0-9]+', display):
        raise ValueError('EXPLICIT_LOCAL_DISPLAY_REQUIRED')
    return ['timeout', '--signal=TERM', '--kill-after=10s', '710s', sys.executable,
        str(source/'tools/run_time_path_awsim_trial.py'), '--deployment', str(deployment),
        '--run-id', run_id, '--display', display, '--config', 'configs/control/time_path_dev.json',
        '--ros-launch', '--max-speed-kmh', str(speeds.max_speed_kmh),
        '--corner-max-speed-kmh', str(speeds.corner_max_speed_kmh),
        *(['--record-video'] if record_video else [])]


def main() -> None:
    source = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deployment', type=Path, default=source.parent)
    parser.add_argument('--run-id', default='codex-time-dev-'+datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S-%f'))
    parser.add_argument('--display', default=os.environ.get('DISPLAY') or ':0')
    parser.add_argument('--max-speed-kmh', type=float, default=20.)
    parser.add_argument('--corner-max-speed-kmh', type=float, default=10.)
    parser.add_argument('--record-video', action='store_true')
    args = parser.parse_args()
    speeds = TimeDevSpeeds(args.max_speed_kmh, args.corner_max_speed_kmh)
    deployment = args.deployment.resolve()
    if (os.name != 'posix' or deployment.parent != Path('/home/graneple/e2e_autonomous')
            or deployment not in source.parents):
        raise ValueError('RUN_MAKE_ON_PREPARED_GRANEPLE_AWSIM_DEPLOYMENT')
    if not (deployment/'install/setup.bash').is_file() or not (deployment/'command_off_best.pt').is_file():
        raise ValueError('PREPARED_ROS_INSTALL_AND_CHECKPOINT_REQUIRED')
    command = make_command(source=source, deployment=deployment, run_id=args.run_id,
        display=args.display, speeds=speeds, record_video=args.record_video)
    print(f'TimePath: max={speeds.max_speed_kmh:g} km/h, corner={speeds.corner_max_speed_kmh:g} km/h; '
          f'output={deployment/args.run_id}', flush=True)
    raise SystemExit(subprocess.run(command, cwd=source).returncode)


if __name__ == '__main__':
    main()
