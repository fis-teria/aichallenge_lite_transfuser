"""Prepare a separate, hash-bound candidate deployment without simulator edits."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import tarfile
import time

ROOT = Path('/home/graneple/e2e_autonomous/time_native_retained_awsim_20260919')
SOURCE = ROOT / 'source_d12bdba'
REPO = Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
SHA = 'e7afdab5d05873de0dbed454e1086b32f4d4f1b67f884a9417a349d7db726ad2'
COMMIT = 'd12bdba09a1be6d0a91a28e18c9127853dfbefa0'


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def save(name: str, value: object) -> None:
    with (ROOT / name).open('x') as stream:
        json.dump(value, stream, indent=2)


assert not subprocess.check_output(['docker', 'ps', '-q'], text=True).strip()
assert shutil.disk_usage(ROOT).free > 2 * 1024**3
archive = ROOT / 'source_d12bdba.tar'
assert sha(archive) == 'b6fae0a2089562e72c7a96e5f427c7233573bf00db2fd8ef2a809f3b5e1db39a'
assert sha(ROOT / 'retained_step0512.pt') == SHA
SOURCE.mkdir()
with tarfile.open(archive) as tf:
    assert all(not m.name.startswith('/') and '..' not in Path(m.name).parts
               and not m.issym() and not m.islnk() for m in tf.getmembers())
    tf.extractall(SOURCE)
(ROOT / 'command_off_best.pt').symlink_to('retained_step0512.pt')
shutil.copyfile('/home/graneple/e2e_autonomous/time_no_gnss_20260918_r2/smoke_dds.xml', ROOT / 'smoke_dds.xml')
config = json.loads((SOURCE / 'configs/control/time_path_dev.json').read_text())
original_sha = config['checkpoint_sha256']
config['checkpoint_sha256'] = SHA
config['checkpoint_epoch'] = 1
override = SOURCE / 'configs/control/time_native_replay_trial.json'
override.write_text(json.dumps(config, indent=2) + '\n')
save('source_verification.json', dict(commit=COMMIT, archive_sha256=sha(archive),
    checkpoint_sha256=SHA, original_checkpoint_sha256=original_sha,
    override_path=str(override.relative_to(SOURCE)), override_sha256=sha(override),
    config_changes=['checkpoint_sha256', 'checkpoint_epoch'], adoption='ISOLATED_TEST_ONLY_NOT_DEFAULT_PROMOTION'))

base = ['docker', 'run', '--rm', '--network', 'none', '--user', '1000:1000',
    '-e', 'ROS_DOMAIN_ID=93', '-e', 'ROS_LOG_DIR=/time/ros_logs',
    '-e', 'CYCLONEDDS_URI=file:///time/smoke_dds.xml',
    '-v', str(ROOT) + ':/time', '-v', str(REPO / 'aichallenge') + ':/aichallenge:ro',
    '--entrypoint', 'bash', 'codex-cartographer-v4-build:20260910', '-lc']


def run(name: str, shell: str, timeout: float) -> None:
    command = base + [shell]
    save(name + '_command.json', command)
    start = time.monotonic()
    print('START ' + name, flush=True)
    with (ROOT / (name + '.log')).open('x') as log:
        code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=timeout).returncode
    save(name + '_exit.json', dict(exit=code, wall_s=time.monotonic() - start))
    print(name + ' exit=' + str(code), flush=True)
    if code:
        print((ROOT / (name + '.log')).read_text()[-7000:], flush=True)
        raise RuntimeError(name)


run('build', 'source /aichallenge/workspace/install/setup.bash && cd /time/' + SOURCE.name +
    '/ros2_ws && colcon --log-base /time/build_log build --packages-select aic_e2e_runtime'
    ' --build-base /time/build --install-base /time/install', 180)
files = []
for src, dst in [
    (SOURCE / 'src/aic_transfuser_lite', ROOT / 'install/aic_e2e_runtime/share/aic_e2e_runtime/python_src/aic_transfuser_lite'),
    (SOURCE / 'ros2_ws/src/aic_e2e_runtime/aic_e2e_runtime', ROOT / 'install/aic_e2e_runtime/lib/python3.10/site-packages/aic_e2e_runtime')]:
    for path in src.rglob('*.py'):
        installed = dst / path.relative_to(src)
        assert sha(path) == sha(installed), str(path)
        files.append(dict(path=str(installed.relative_to(ROOT)), sha256=sha(installed)))
package = ROOT / 'install/aic_e2e_runtime/share/aic_e2e_runtime'
assert sha(SOURCE / 'ros2_ws/src/aic_e2e_runtime/launch/time_path_awsim.launch.py') == sha(package / 'launch/time_path_awsim.launch.py')
# This isolated installation's default points to the candidate. The canonical
# source default and every earlier deployment/entrypoint remain unchanged.
shutil.copyfile(override, package / 'config/time_path_dev.json')
save('install_verification.json', dict(status='PASS', files=files,
    generated_default_config_sha256=sha(package / 'config/time_path_dev.json')))
prefix = 'source /aichallenge/workspace/install/setup.bash && source /time/install/setup.bash && '
run('connection_smoke', prefix + 'timeout --signal=TERM --kill-after=10s 110s python3 /time/' +
    SOURCE.name + '/tools/check_time_ros_connection.py --checkpoint /time/command_off_best.pt'
    ' --checkpoint-sha256 ' + SHA + ' --trial-config /time/' + SOURCE.name +
    '/configs/control/time_native_replay_trial.json --output /time/connection_smoke', 130)
run('launch_smoke', prefix + 'timeout --signal=TERM --kill-after=10s 70s python3 /time/' +
    SOURCE.name + '/tools/check_time_dev_launch.py --checkpoint /time/command_off_best.pt'
    ' --output /time/launch_smoke', 90)
save('ready.json', dict(status='READY_FOR_BOUNDED_AWSIM_TEST', checkpoint_sha256=SHA,
    source_commit=COMMIT, checked_python_files=len(files),
    connection=json.loads((ROOT / 'connection_smoke/summary.json').read_text()),
    launch=json.loads((ROOT / 'launch_smoke/summary.json').read_text())))
print('READY_FOR_BOUNDED_AWSIM_TEST', flush=True)
