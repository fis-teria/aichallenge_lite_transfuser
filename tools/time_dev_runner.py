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
from aic_transfuser_lite.runtime.awsim_trial_session import validate_npc_count
from aic_transfuser_lite.runtime.awsim_traffic import traffic_domains


def make_command(*, source: Path, deployment: Path, run_id: str, display: str,
                 speeds: TimeDevSpeeds, record_video: bool, npcs: int = 0, pp_vehicles: int = 0,
                 slam_mppi: bool = False) -> list[str]:
    """Explicit argument vector, no shell interpolation; outer timeout in seconds."""
    if not re.fullmatch(r'codex-time-[a-z0-9-]+', run_id):
        raise ValueError('INVALID_OWNED_RUN_ID')
    if not re.fullmatch(r':[0-9]+', display):
        raise ValueError('EXPLICIT_LOCAL_DISPLAY_REQUIRED')
    validate_npc_count(npcs)
    traffic_domains(pp_vehicles, npcs)
    if slam_mppi:
        selected = (speeds.max_speed_kmh, speeds.corner_max_speed_kmh)
        if selected not in ((5., 5.), (20., 15.)) or npcs or pp_vehicles:
            raise ValueError('SLAM_MPPI_REQUIRES_5_5_OR_20_15_STATIC_SINGLE_EGO')
        config = 'time_path_slam_mppi_20_15_15_recovery.json' if selected == (20., 15.) else 'time_path_slam_mppi.json'
        return ['timeout', '--signal=TERM', '--kill-after=10s', '710s', sys.executable,
            str(source/'tools/run_time_path_awsim_trial.py'), '--deployment', str(deployment),
            '--run-id', run_id, '--display', display, '--config', 'configs/control/'+config,
            '--slam-obstacles', *(['--record-video'] if record_video else [])]
    return ['timeout', '--signal=TERM', '--kill-after=10s', '710s', sys.executable,
        str(source/'tools/run_time_path_awsim_trial.py'), '--deployment', str(deployment),
        '--run-id', run_id, '--display', display, '--config', 'configs/control/time_path_dev.json',
        '--npcs', str(npcs), '--pp-vehicles', str(pp_vehicles), '--ros-launch', '--max-speed-kmh', str(speeds.max_speed_kmh),
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
    parser.add_argument('--slam-mppi', action='store_true', help='Static-obstacle MPPI trial: 5/5 or 20/15 km/h, latter avoidance cap 15')
    parser.add_argument('--npcs', type=int, choices=range(4), default=0,
                        help='Built-in AWSIM NPC karts; only ego has an E2E controller')
    parser.add_argument('--pp-vehicles', type=int, choices=range(4), default=0,
                        help='Existing PP background cars on domains 2..4; exclusive with built-in NPCs')
    args = parser.parse_args()
    speeds = TimeDevSpeeds(args.max_speed_kmh, args.corner_max_speed_kmh)
    deployment = args.deployment.resolve()
    if (os.name != 'posix' or deployment.parent != Path('/home/graneple/e2e_autonomous')
            or deployment not in source.parents):
        raise ValueError('RUN_MAKE_ON_PREPARED_GRANEPLE_AWSIM_DEPLOYMENT')
    if not (deployment/'install/setup.bash').is_file() or not (deployment/'command_off_best.pt').is_file():
        raise ValueError('PREPARED_ROS_INSTALL_AND_CHECKPOINT_REQUIRED')
    command = make_command(source=source, deployment=deployment, run_id=args.run_id,
        display=args.display, speeds=speeds, record_video=args.record_video, npcs=args.npcs,
        pp_vehicles=args.pp_vehicles, slam_mppi=args.slam_mppi)
    print(f'TimePath: max={speeds.max_speed_kmh:g} km/h, corner={speeds.corner_max_speed_kmh:g} km/h; '
          f'NPCs={args.npcs}; PP cars={args.pp_vehicles}; output={deployment/args.run_id}', flush=True)
    raise SystemExit(subprocess.run(command, cwd=source).returncode)


if __name__ == '__main__':
    main()
