"""Finish only the named completed trial, then verify and evaluate in WSL."""
from pathlib import Path
import hashlib
import json
import subprocess
from manage import remote, run, HOST, ROOT, WSL, HERE


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


remote((HERE/'finalize_remote.py').read_text(), timeout=180)
remote((HERE/'seal_archive.py').read_text(), timeout=180)
native = Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered')/WSL.lstrip('/')
names = ['shipping.json', 'raw_trial.tar.gz', 'post_environment.json', 'runner_start.json', 'runner_exit.json',
         'source_manifest.json', 'source_verification.json', 'install_verification.json', 'pre_environment.json',
         'build_exit.json', 'smoke_exit.json', 'launch_smoke_exit.json', 'dev_entrypoint.json', 'make_parameter_smoke.json', 'awsim_reference_snapshot.json', 'awsim_preserved.json']
checks = json.loads(remote('''
from pathlib import Path
import hashlib,json
root=Path('''+repr(ROOT)+''')
names='''+repr(names)+'''
def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024),b''):
            digest.update(block)
    return digest.hexdigest()
print(json.dumps({name:dict(bytes=(root/name).stat().st_size,sha256=sha(root/name)) for name in names}))
''', timeout=60))
for name in names:
    assert not (native/name).exists(), name
run(['scp', *[HOST+':'+ROOT+'/'+name for name in names], str(native)], timeout=180)
for name, check in checks.items():
    path=native/name
    assert path.stat().st_size == check['bytes'] and sha(path) == check['sha256'],name
run(['scp', HOST+':'+ROOT+'/smoke/summary.json', str(native/'ros_smoke_summary.json')])
run(['scp', HOST+':'+ROOT+'/launch_smoke/summary.json', str(native/'ros_launch_smoke_summary.json')])
(native/'deployment_transfer_verification.json').write_text(json.dumps(dict(status='PASS',files=checks),indent=2))
remote((HERE/'evaluate_native.py').read_text(), host='codex-wsl', lock=True, timeout=300)
