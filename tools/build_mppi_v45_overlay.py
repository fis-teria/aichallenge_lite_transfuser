"""Build the vendored teacher and LiDAR adapter without altering a host overlay.

Linux/Docker only; run from the deployed source tree. Runtime uses seconds and
bytes, no vehicle motion. A new install directory is required for each build.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--awsim-repo', type=Path, required=True)
    ap.add_argument('--runtime', type=Path, required=True)
    ap.add_argument('--image', default='aichallenge-2025-dev:latest')
    ap.add_argument('--timeout-s', type=int, default=1800)
    args = ap.parse_args()
    source = Path(__file__).resolve().parents[1]
    root = source/'integrations/mppi_v45'
    manifest = json.loads((root/'source_manifest.json').read_text())
    for row in manifest['files']:
        assert sha(root/row['path']) == row['sha256'], row['path']
    assert 60 <= args.timeout_s <= 3600
    assert not (args.runtime/'install').exists(), 'Preserve existing installed artifacts'
    assert shutil.disk_usage(args.runtime.parent).free > 3*2**30
    args.runtime.mkdir(parents=True, exist_ok=True)
    image_id = subprocess.check_output(['docker', 'image', 'inspect', '-f', '{{.Id}}', args.image], text=True).strip()
    name = 'codex-mppi-v45-build-' + hashlib.sha256(str(args.runtime).encode()).hexdigest()[:8]
    command = ['docker','run','--rm','--name',name,'--network','none','--user',f'{os.getuid()}:{os.getgid()}',
        '--tmpfs','/teacher-build:rw,exec,size=5g,mode=1777', '-e','HOME=/tmp',
        '-v',f'{args.awsim_repo.resolve()}/aichallenge:/aichallenge:ro',
        '-v',f'{source}:/source:ro','-v',f'{args.runtime.resolve()}:/runtime:rw',
        '--entrypoint','bash',image_id,'/source/integrations/mppi_v45/build_overlay.bash']
    (args.runtime/'build-inputs.json').write_text(json.dumps({'command':command,
        'source_archive_sha256':manifest['source_archive_sha256'],
        'source_manifest_sha256':sha(root/'source_manifest.json'),
        'collection_revision':manifest.get('collection_revision'), 'image_id':image_id},indent=2)+'\n')
    started = time.time()
    with (args.runtime/'build.log').open('x') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        try:
            code = process.wait(timeout=args.timeout_s)
        except subprocess.TimeoutExpired:
            subprocess.run(['docker','stop','-t','20',name],check=True,timeout=35)
            code = process.wait(timeout=10)
    receipt = {'exit_code':code,'seconds':time.time()-started,'image_id':image_id}
    (args.runtime/'build-result.json').write_text(json.dumps(receipt,indent=2)+'\n')
    if code == 0:
        files = {'node':'reference_space_mppi_planner/lib/reference_space_mppi_planner/reference_space_mppi_node',
                 'core':'reference_space_mppi_planner/lib/libreference_space_mppi_core.so',
                 'config':'reference_space_mppi_planner/share/reference_space_mppi_planner/config/reference_space_mppi.param.yaml'}
        for name in ('core', 'node', 'collection_intent'):
            matches = list((args.runtime/'install/mppi_recovery_controller/lib').glob(
                'python*/site-packages/mppi_recovery_controller/'+name+'.py'))
            assert len(matches) == 1, (name, matches)
            files['recovery_'+name] = matches[0].relative_to(args.runtime/'install').as_posix()
        files['teacher_launch'] = 'aic_lidar_v2x/share/aic_lidar_v2x/launch/teacher_v45.launch.py'
        identity = {'teacher':'MPPI_SIM_V45','source_archive_sha256':manifest['source_archive_sha256'],
                    'collection_revision':manifest.get('collection_revision'),
                    'source_manifest_sha256':sha(root/'source_manifest.json'),
                    'files':{key:{'path':rel,'sha256':sha(args.runtime/'install'/rel)} for key,rel in files.items()}}
        (args.runtime/'runtime-identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    print(json.dumps(receipt),flush=True)
    raise SystemExit(code)


if __name__ == '__main__':
    main()
