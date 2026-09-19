"""Run one candidate lap, record provenance, preserve earlier deployment state."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import time
import sys

ROOT = Path('/home/graneple/e2e_autonomous/time_native_retained_awsim_20260919')
SOURCE = ROOT / 'source_d12bdba'
REPO = Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
KIND = sys.argv[1]
assert KIND in ('cone', 'box')
RUN_ID = 'codex-time-retained-' + KIND + '-01'


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def save(name: str, value: object) -> None:
    with (ROOT / (KIND + '_' + name)).open('x') as stream:
        json.dump(value, stream, indent=2)


def snapshot() -> dict:
    assets = REPO / 'aichallenge/simulator/AWSIM'
    protected = [
        Path('/home/graneple/e2e_autonomous/time_path_dev_20260917/Makefile'),
        Path('/home/graneple/e2e_autonomous/time_no_gnss_20260918_r2/command_off_best.pt'),
        REPO / 'aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/autoware.rviz']
    return dict(assets={str(p.relative_to(assets)): sha(p) for p in sorted(assets.rglob('*')) if p.is_file()},
        protected={str(p): sha(p) for p in protected},
        repo_diff_sha256=hashlib.sha256(subprocess.check_output(['git', '-C', str(REPO), 'diff', '--binary', 'HEAD'])).hexdigest(),
        active_containers=subprocess.check_output(['docker', 'ps', '--format', '{{json .}}'], text=True),
        free_bytes=shutil.disk_usage(ROOT).free)


ready = json.loads((ROOT / 'ready.json').read_text())
assert ready['status'] == 'READY_FOR_BOUNDED_AWSIM_TEST'
assert not subprocess.check_output(['docker', 'ps', '-q'], text=True).strip()
assert shutil.disk_usage(ROOT).free > 2 * 1024**3
for item in json.loads((ROOT / 'install_verification.json').read_text())['files']:
    assert sha(ROOT / item['path']) == item['sha256'], item['path']
config = json.loads((SOURCE / 'configs/control/time_native_replay_trial.json').read_text())
assert sha(ROOT / 'command_off_best.pt') == config['checkpoint_sha256']
before = snapshot()
save('trial_preflight.json', dict(**before, checkpoint_sha256=config['checkpoint_sha256'],
    authorization='User requested AWSIM test 2026-09-19. One fixed obstacle, ego-only lap, max10/corner10 km/h; no default model promotion.',
    offline_gate='CANDIDATE_FOR_AWSIM_TEST: retained_step0512 passed all existing retention and native-fit gates; isolated candidate test.'))
command = ['timeout', '--signal=TERM', '--kill-after=10s', '710s', 'python3',
    str(SOURCE / 'tools/run_time_path_awsim_trial.py'), '--deployment', str(ROOT),
    '--run-id', RUN_ID, '--display', ':1', '--config', 'configs/control/time_native_replay_trial.json',
    '--ros-launch', '--max-speed-kmh', '10', '--corner-max-speed-kmh', '10',
    '--npcs', '0', '--pp-vehicles', '0', '--record-video',
    '--static-obstacle-scenario', 'configs/scenarios/time_avoidance_single_' + KIND + '.yaml']
save('trial_command.json', command)
started = time.monotonic()
print('START ' + RUN_ID, flush=True)
with (ROOT / (KIND + '_trial_runner.log')).open('x') as log:
    process = subprocess.Popen(command, cwd=SOURCE, env=dict(os.environ, DISPLAY=':1'),
                               stdout=log, stderr=subprocess.STDOUT)
    save('trial_pid.json', dict(pid=process.pid, run_id=RUN_ID))
    code = process.wait()
save('trial_exit.json', dict(exit=code, wall_s=time.monotonic()-started))
after = snapshot()
save('trial_postflight.json', dict(**after, assets_unchanged=before['assets'] == after['assets'],
    protected_unchanged=before['protected'] == after['protected'],
    repo_diff_unchanged=before['repo_diff_sha256'] == after['repo_diff_sha256']))
host_file = ROOT / RUN_ID / 'host_result.json'
host = json.loads(host_file.read_text()) if host_file.exists() else {}
print(json.dumps(dict(exit=code, wall_s=time.monotonic()-started, status=host.get('status'),
    error=host.get('error'), laps=host.get('judge_laps'), sections=host.get('judge_section_events'),
    assets_unchanged=before['assets'] == after['assets'], protected_unchanged=before['protected'] == after['protected'])), flush=True)
raise SystemExit(code)
